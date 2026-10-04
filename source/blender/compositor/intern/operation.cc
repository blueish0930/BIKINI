/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include <limits>
#include <memory>
#include <string>

#include "BLI_map.hh"
#include "BLI_string_ref.hh"
#include "BLI_vector.hh"

#include "COM_context.hh"
#include "COM_conversion_operation.hh"
#include "COM_domain.hh"
#include "COM_input_descriptor.hh"
#include "COM_operation.hh"
#include "COM_realize_on_domain_operation.hh"
#include "COM_result.hh"
#include "COM_simple_operation.hh"

namespace blender::compositor {

Operation::Operation(Context &context) : context_(context) {}

Operation::~Operation() = default;

void Operation::evaluate()
{
  this->evaluate_input_processors();
  this->execute();
  this->log_data();
  this->release_inputs();
  this->context().evaluate_operation_post();
}

Result &Operation::get_input(StringRef identifier) const
{
  return *results_mapped_to_inputs_.lookup_as(identifier);
}

bool Operation::has_input(StringRef identifier) const
{
  return results_mapped_to_inputs_.contains_as(identifier);
}

Result *Operation::try_get_input(StringRef identifier) const
{
  Result *const *mapped = results_mapped_to_inputs_.lookup_ptr_as(identifier);
  if (mapped == nullptr || *mapped == nullptr) {
    return nullptr;
  }
  if (intptr_t(*mapped) < 4096) {
    return nullptr;
  }
  return *mapped;
}

Result &Operation::get_result(StringRef identifier)
{
  return results_.lookup(identifier);
}

bool Operation::has_result(StringRef identifier) const
{
  return results_.contains_as(identifier);
}

void Operation::map_input_to_result(StringRef identifier, Result *result)
{
  /* Own the key: callers may pass temporary multi-input identifiers (e.g. "Stamp__1").
   * Use add_overwrite so reconnect does not assert. Never store a null Result* — mute/bypass
   * can otherwise crash in get_input / is_allocated (read at offset 0xA). */
  if (result == nullptr || intptr_t(result) < 4096) {
    results_mapped_to_inputs_.remove_as(identifier);
    return;
  }
  results_mapped_to_inputs_.add_overwrite(std::string(identifier), result);
}

void Operation::free_results()
{
  for (Result &result : results_.values()) {
    result.free();
  }
}

void Operation::pin_results()
{
  /* Large enough that multipass body re-releases cannot free exterior textures, small enough
   * to avoid signed overflow if many consumers still decrement. */
  constexpr int pin_count = 1 << 20;
  for (Result &result : results_.values()) {
    if (result.is_allocated()) {
      result.set_reference_count(pin_count);
    }
  }
}

Context &Operation::context() const
{
  return context_;
}

Domain Operation::compute_domain()
{
  /* Default to an identity domain in case no domain input was found, most likely because all
   * inputs are single values. */
  Domain operation_domain = Domain::identity();
  int current_domain_priority = std::numeric_limits<int>::max();

  /* Only walk mapped inputs. Multi-input pre-declare may list slots that were not mapped yet;
   * looking them up in get_input would assert/crash (e.g. Point Stamp reconnect). */
  for (const auto item : results_mapped_to_inputs_.items()) {
    if (!item.value) {
      continue;
    }
    if (!input_descriptors_.contains_as(item.key)) {
      continue;
    }
    const Result &result = *item.value;
    const InputDescriptor &descriptor = input_descriptors_.lookup_as(item.key);

    /* A single value input can't be a domain input. */
    if (result.is_single_value() || descriptor.expects_single_value) {
      continue;
    }

    /* An input that skips operation domain realization can't be a domain input. */
    if (descriptor.realization_mode != InputRealizationMode::OperationDomain) {
      continue;
    }

    /* Notice that the lower the domain priority value is, the higher the priority is, hence the
     * less than comparison. */
    if (descriptor.domain_priority < current_domain_priority) {
      operation_domain = result.domain();
      current_domain_priority = descriptor.domain_priority;
    }
  }

  return operation_domain;
}

void Operation::evaluate_input_processors()
{
  /* Each input processor type is added to all inputs entirely before the next type. This is done
   * because the construction of the input processors may depend on the result of previous input
   * processors for all inputs. For instance, the realize on domain input processor considers the
   * value of all inputs, so previous input processors for all inputs needs to be added and
   * evaluated first. */

  /* Snapshot keys first: processors rewrite results_mapped_to_inputs_ values; multi-input
   * reconnect must not iterate a mutating map of dangling temporary identifiers. */
  Vector<std::string> mapped_identifiers;
  mapped_identifiers.reserve(results_mapped_to_inputs_.size());
  for (const std::string &identifier : results_mapped_to_inputs_.keys()) {
    mapped_identifiers.append(identifier);
  }

  for (const std::string &identifier : mapped_identifiers) {
    Result *const *mapped = results_mapped_to_inputs_.lookup_ptr_as(identifier);
    if (mapped == nullptr || *mapped == nullptr || !input_descriptors_.contains_as(identifier)) {
      continue;
    }
    SimpleOperation *conversion = ConversionOperation::construct_if_needed(
        this->context(), **mapped, this->get_input_descriptor(identifier));
    this->add_and_evaluate_input_processor(identifier, conversion);
  }

  const Domain domain = this->compute_domain();
  for (const std::string &identifier : mapped_identifiers) {
    Result *const *mapped = results_mapped_to_inputs_.lookup_ptr_as(identifier);
    if (mapped == nullptr || *mapped == nullptr || !input_descriptors_.contains_as(identifier)) {
      continue;
    }
    SimpleOperation *realize_on_domain = RealizeOnDomainOperation::construct_if_needed(
        this->context(), **mapped, this->get_input_descriptor(identifier), domain);
    this->add_and_evaluate_input_processor(identifier, realize_on_domain);
  }
}

void Operation::log_data() {};

void Operation::populate_result(StringRef identifier, const ResultType type)
{
  results_.add_new(identifier, this->context().create_result(type));
}

void Operation::declare_input_descriptor(StringRef identifier, InputDescriptor descriptor)
{
  input_descriptors_.add_new(identifier, descriptor);
}

InputDescriptor &Operation::get_input_descriptor(StringRef identifier)
{
  return input_descriptors_.lookup(identifier);
}

void Operation::allocate_default_remaining_outputs()
{
  for (Result &result : results_.values()) {
    if (result.should_compute() && !result.is_allocated()) {
      result.allocate_invalid();
    }
  }
}

void Operation::add_and_evaluate_input_processor(StringRef identifier, SimpleOperation *processor)
{
  /* Allow null inputs to facilitate construct_if_needed pattern of addition. For instance, see the
   * implementation of the evaluate_input_processors method. */
  if (!processor) {
    return;
  }

  const std::string key(identifier);
  ProcessorsVector &processors = input_processors_.lookup_or_add_default(key);

  /* Get the result that should serve as the input for the processor. This is either the result
   * mapped to the input or the result of the last processor depending on whether this is the first
   * processor or not. */
  Result &result = processors.is_empty() ? this->get_input(key) : processors.last()->get_result();

  /* Map the input result of the processor and add it to the processors vector. */
  processor->map_input_to_result(&result);
  processors.append(std::unique_ptr<SimpleOperation>(processor));

  /* Switch the result mapped to the input to be the output result of the processor. */
  results_mapped_to_inputs_.lookup(key) = &processor->get_result();

  processor->evaluate();
}

void Operation::release_inputs()
{
  for (Result *result : results_mapped_to_inputs_.values()) {
    if (result == nullptr) {
      continue;
    }
    result->release();
  }
}

}  // namespace blender::compositor
