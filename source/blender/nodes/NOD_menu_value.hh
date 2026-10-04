/* SPDX-FileCopyrightText: 2025 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup nodes
 */

#pragma once

#include "BLI_hash.hh"
#include "BLI_math_bits.hh"
#include "BLI_unique_hash.hh"
#include "BLI_vector.hh"

namespace blender::nodes {

/**
 * Don't use integer for menus directly, so that there is a each static single value type maps to
 * exactly one socket type. Also it avoids accidentally casting the menu value to other types.
 *
 * When #multi_selection is false, #value is the selected item identifier.
 * When #multi_selection is true, #value is a bit-field where bit `1 << identifier` means the item
 * with that identifier is selected. Identifiers must be in [0, 30] for multi-selection.
 */
struct MenuValue {
  int value = 0;
  bool multi_selection = false;

  MenuValue() = default;
  explicit MenuValue(const int value) : value(value) {}
  MenuValue(const int value, const bool multi_selection)
      : value(value), multi_selection(multi_selection)
  {
  }

  template<typename EnumT>
  MenuValue(const EnumT value)
    requires(std::is_enum_v<EnumT>)
      : value(int(value))
  {
  }

  /** Bit used for a multi-selection item with the given identifier. */
  static int multi_bit_for_identifier(const int identifier)
  {
    if (identifier < 0 || identifier >= 31) {
      return 0;
    }
    return 1 << identifier;
  }

  /** Convert a single-selection identifier to multi-selection bit-field form. */
  static int single_to_multi_bits(const int identifier)
  {
    return multi_bit_for_identifier(identifier);
  }

  /** Convert multi-selection bit-field to a single identifier (lowest set bit), or 0 if empty. */
  static int multi_bits_to_single(const int bits)
  {
    if (bits == 0) {
      return 0;
    }
    return int(bitscan_forward_uint(uint(bits)));
  }

  bool item_is_selected(const int identifier) const
  {
    if (multi_selection) {
      const int bit = multi_bit_for_identifier(identifier);
      return bit != 0 && (value & bit) != 0;
    }
    return value == identifier;
  }

  /**
   * Index of the first item selected by this menu, or -1.
   * `identifier_fn(i)` returns the identifier of item `i`.
   * Compositor keeps that first match; it does not pack a list.
   */
  template<typename IdentifierFn>
  int first_selected_enum_index(const int items_num, IdentifierFn identifier_fn) const
  {
    for (int i = 0; i < items_num; i++) {
      if (this->item_is_selected(int(identifier_fn(i)))) {
        return i;
      }
    }
    return -1;
  }

  /**
   * Turn this value into a bit-field when the menu source is multi-selection.
   * Does not guess from the integer itself: single-select identifiers are 0, 1, 2, 3, ...
   * and values such as 3, 5, 6, 7 have more than one bit set. Treating those as bits
   * highlights the wrong items and leaves the selected socket unused.
   */
  void ensure_multi_selection(const bool source_is_multi)
  {
    if (source_is_multi) {
      multi_selection = true;
    }
  }

  /**
   * Single-select: #value is an item identifier, even when that identifier has several bits.
   * Multi-select: #value is a bit-field. Use bits only when this value or #source_is_multi
   * already says so, or when the integer matches no item (a bit-field whose flag was dropped).
   * Value 0 without multi is identifier 0, not an empty selection.
   */
  template<typename IdentifierFn>
  MenuValue normalized(const bool source_is_multi,
                       const int items_num,
                       IdentifierFn identifier_fn) const
  {
    if (this->multi_selection || source_is_multi) {
      return MenuValue(this->value, true);
    }
    for (int i = 0; i < items_num; i++) {
      if (int(identifier_fn(i)) == this->value) {
        return MenuValue(this->value, false);
      }
    }
    if (this->value != 0) {
      return MenuValue(this->value, true);
    }
    return MenuValue(this->value, false);
  }

  /** Identifiers of all selected items (order by identifier ascending). */
  Vector<int> selected_identifiers() const
  {
    Vector<int> result;
    if (multi_selection) {
      for (int id = 0; id < 31; id++) {
        if (value & multi_bit_for_identifier(id)) {
          result.append(id);
        }
      }
    }
    else {
      result.append(value);
    }
    return result;
  }

  uint64_t hash() const
  {
    return get_default_hash(this->value, this->multi_selection);
  }

  void hash_unique(UniqueHashBytes &hash) const
  {
    hash_unique_default(this->value, hash);
    hash_unique_default(this->multi_selection, hash);
  }

  friend bool operator==(const MenuValue &a, const MenuValue &b) = default;
};

}  // namespace blender::nodes
