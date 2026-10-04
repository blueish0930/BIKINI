/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup bke
 *
 * Encrypt node-group internals so a .blend file does not contain plaintext geometry
 * (or other) node graphs. The graph is stored as an AEAD-encrypted mini-blend.
 */

#include <algorithm>
#include <cstdio>
#include <cstring>
#include <string>

#ifdef _WIN32
#  ifndef WIN32_LEAN_AND_MEAN
#    define WIN32_LEAN_AND_MEAN
#  endif
#  include <windows.h>
#  include <bcrypt.h>
#endif

#include "MEM_guardedalloc.h"

#include "DNA_node_types.h"
#include "DNA_space_enums.h"

#include "BLI_fileops.hh"
#include "BLI_hash_md5.hh"
#include "BLI_listbase.hh"
#include "BLI_map.hh"
#include "BLI_path_utils.hh"
#include "BLI_set.hh"
#include "BLI_string.hh"
#include "BLI_string_utf8.hh"
#include "BLI_tempfile.hh"
#include "BLI_utildefines.hh"
#include "BLI_vector.hh"
#include "BLI_vector_set.hh"

#include "BKE_blendfile.hh"
#include "BKE_global.hh"
#include "BKE_idtype.hh"
#include "BKE_lib_id.hh"
#include "BKE_lib_query.hh"
#include "BKE_library.hh"
#include "BKE_main.hh"
#include "BKE_main_invariants.hh"
#include "BKE_node.hh"
#include "BKE_node_runtime.hh"
#include "BKE_node_tree_reference_lifetimes.hh"
#include "BKE_node_tree_update.hh"
#include "BKE_report.hh"

#include "BLO_readfile.hh"
#include "BLO_writefile.hh"

#include "NOD_node_declaration.hh"

#include "CLG_log.h"

namespace blender::bke {
namespace {

CLG_LogRef LOG = {"node.lock"};

constexpr int kLockKeyLen = 32;
constexpr int kLockSaltLen = 16;
constexpr int kLockNonceLen = 12;
constexpr int kLockTagLen = 32; /* HMAC-SHA256 */
constexpr int kLockKdfIters = 100000;
constexpr char kLockMagic[4] = {'N', 'T', 'L', '2'};

/**
 * Black-box at-rest encryption key, embedded in the binary. Locked groups are encrypted
 * with this key when written to a .blend, so the file contains no plaintext node graph
 * and regular Blender (which skips the unknown DNA fields) sees an empty group. BIKINI
 * decrypts transparently on load so locked groups still evaluate without a password.
 *
 * This is obfuscation, not real security: anyone with this binary (or its sources) can
 * extract the key. The user-chosen lock password separately gates viewing/editing.
 */
constexpr unsigned char kBlackboxKey[kLockKeyLen] = {
    0x42, 0x49, 0x4b, 0x49, 0x4e, 0x49, 0x2d, 0x4c, /* "BIKINI-L" */
    0x4f, 0x43, 0x4b, 0x2d, 0x42, 0x4c, 0x41, 0x43, /* "OCK-BLAC" */
    0x4b, 0x42, 0x4f, 0x58, 0x2d, 0x4b, 0x45, 0x59, /* "KBOX-KEY" */
    0x2d, 0x32, 0x30, 0x32, 0x36, 0x2d, 0x30, 0x39, /* "-2026-09" */
};

void secure_zero(void *ptr, const size_t n)
{
  volatile unsigned char *p = static_cast<volatile unsigned char *>(ptr);
  for (size_t i = 0; i < n; i++) {
    p[i] = 0;
  }
}

bool fill_random(void *buf, const size_t n)
{
#ifdef _WIN32
  return BCryptGenRandom(nullptr,
                         static_cast<PUCHAR>(buf),
                         ULONG(n),
                         BCRYPT_USE_SYSTEM_PREFERRED_RNG) == 0;
#else
  FILE *fp = fopen("/dev/urandom", "rb");
  if (!fp) {
    return false;
  }
  const size_t read = fread(buf, 1, n, fp);
  fclose(fp);
  return read == n;
#endif
}

bool secure_equal(const void *a, const void *b, const size_t n)
{
  const unsigned char *pa = static_cast<const unsigned char *>(a);
  const unsigned char *pb = static_cast<const unsigned char *>(b);
  unsigned char diff = 0;
  for (size_t i = 0; i < n; i++) {
    diff |= pa[i] ^ pb[i];
  }
  return diff == 0;
}

/* -------------------------------------------------------------------- */
/** \name SHA-256
 * \{ */

struct SHA256Ctx {
  uint32_t state[8];
  uint64_t bitlen;
  unsigned char data[64];
  uint32_t datalen;
};

uint32_t rotr32(const uint32_t x, const uint32_t n)
{
  return (x >> n) | (x << (32 - n));
}

void sha256_transform(SHA256Ctx &ctx, const unsigned char data[64])
{
  static const uint32_t k[64] = {
      0x428a2f98, 0x71374491, 0xb5c0fbcf, 0xe9b5dba5, 0x3956c25b, 0x59f111f1, 0x923f82a4,
      0xab1c5ed5, 0xd807aa98, 0x12835b01, 0x243185be, 0x550c7dc3, 0x72be5d74, 0x80deb1fe,
      0x9bdc06a7, 0xc19bf174, 0xe49b69c1, 0xefbe4786, 0x0fc19dc6, 0x240ca1cc, 0x2de92c6f,
      0x4a7484aa, 0x5cb0a9dc, 0x76f988da, 0x983e5152, 0xa831c66d, 0xb00327c8, 0xbf597fc7,
      0xc6e00bf3, 0xd5a79147, 0x06ca6351, 0x14292967, 0x27b70a85, 0x2e1b2138, 0x4d2c6dfc,
      0x53380d13, 0x650a7354, 0x766a0abb, 0x81c2c92e, 0x92722c85, 0xa2bfe8a1, 0xa81a664b,
      0xc24b8b70, 0xc76c51a3, 0xd192e819, 0xd6990624, 0xf40e3585, 0x106aa070, 0x19a4c116,
      0x1e376c08, 0x2748774c, 0x34b0bcb5, 0x391c0cb3, 0x4ed8aa4a, 0x5b9cca4f, 0x682e6ff3,
      0x748f82ee, 0x78a5636f, 0x84c87814, 0x8cc70208, 0x90befffa, 0xa4506ceb, 0xbef9a3f7,
      0xc67178f2};

  uint32_t m[64];
  for (int i = 0, j = 0; i < 16; i++, j += 4) {
    m[i] = (uint32_t(data[j]) << 24) | (uint32_t(data[j + 1]) << 16) |
           (uint32_t(data[j + 2]) << 8) | uint32_t(data[j + 3]);
  }
  for (int i = 16; i < 64; i++) {
    const uint32_t s0 = rotr32(m[i - 15], 7) ^ rotr32(m[i - 15], 18) ^ (m[i - 15] >> 3);
    const uint32_t s1 = rotr32(m[i - 2], 17) ^ rotr32(m[i - 2], 19) ^ (m[i - 2] >> 10);
    m[i] = m[i - 16] + s0 + m[i - 7] + s1;
  }

  uint32_t a = ctx.state[0];
  uint32_t b = ctx.state[1];
  uint32_t c = ctx.state[2];
  uint32_t d = ctx.state[3];
  uint32_t e = ctx.state[4];
  uint32_t f = ctx.state[5];
  uint32_t g = ctx.state[6];
  uint32_t h = ctx.state[7];

  for (int i = 0; i < 64; i++) {
    const uint32_t S1 = rotr32(e, 6) ^ rotr32(e, 11) ^ rotr32(e, 25);
    const uint32_t ch = (e & f) ^ ((~e) & g);
    const uint32_t temp1 = h + S1 + ch + k[i] + m[i];
    const uint32_t S0 = rotr32(a, 2) ^ rotr32(a, 13) ^ rotr32(a, 22);
    const uint32_t maj = (a & b) ^ (a & c) ^ (b & c);
    const uint32_t temp2 = S0 + maj;
    h = g;
    g = f;
    f = e;
    e = d + temp1;
    d = c;
    c = b;
    b = a;
    a = temp1 + temp2;
  }

  ctx.state[0] += a;
  ctx.state[1] += b;
  ctx.state[2] += c;
  ctx.state[3] += d;
  ctx.state[4] += e;
  ctx.state[5] += f;
  ctx.state[6] += g;
  ctx.state[7] += h;
}

void sha256_init(SHA256Ctx &ctx)
{
  ctx.datalen = 0;
  ctx.bitlen = 0;
  ctx.state[0] = 0x6a09e667;
  ctx.state[1] = 0xbb67ae85;
  ctx.state[2] = 0x3c6ef372;
  ctx.state[3] = 0xa54ff53a;
  ctx.state[4] = 0x510e527f;
  ctx.state[5] = 0x9b05688c;
  ctx.state[6] = 0x1f83d9ab;
  ctx.state[7] = 0x5be0cd19;
}

void sha256_update(SHA256Ctx &ctx, const unsigned char *data, const size_t len)
{
  for (size_t i = 0; i < len; i++) {
    ctx.data[ctx.datalen++] = data[i];
    if (ctx.datalen == 64) {
      sha256_transform(ctx, ctx.data);
      ctx.bitlen += 512;
      ctx.datalen = 0;
    }
  }
}

void sha256_final(SHA256Ctx &ctx, unsigned char hash[32])
{
  uint32_t i = ctx.datalen;
  if (ctx.datalen < 56) {
    ctx.data[i++] = 0x80;
    while (i < 56) {
      ctx.data[i++] = 0x00;
    }
  }
  else {
    ctx.data[i++] = 0x80;
    while (i < 64) {
      ctx.data[i++] = 0x00;
    }
    sha256_transform(ctx, ctx.data);
    memset(ctx.data, 0, 56);
  }

  ctx.bitlen += uint64_t(ctx.datalen) * 8;
  ctx.data[63] = ctx.bitlen;
  ctx.data[62] = ctx.bitlen >> 8;
  ctx.data[61] = ctx.bitlen >> 16;
  ctx.data[60] = ctx.bitlen >> 24;
  ctx.data[59] = ctx.bitlen >> 32;
  ctx.data[58] = ctx.bitlen >> 40;
  ctx.data[57] = ctx.bitlen >> 48;
  ctx.data[56] = ctx.bitlen >> 56;
  sha256_transform(ctx, ctx.data);

  for (i = 0; i < 4; i++) {
    hash[i] = (ctx.state[0] >> (24 - i * 8)) & 0xff;
    hash[i + 4] = (ctx.state[1] >> (24 - i * 8)) & 0xff;
    hash[i + 8] = (ctx.state[2] >> (24 - i * 8)) & 0xff;
    hash[i + 12] = (ctx.state[3] >> (24 - i * 8)) & 0xff;
    hash[i + 16] = (ctx.state[4] >> (24 - i * 8)) & 0xff;
    hash[i + 20] = (ctx.state[5] >> (24 - i * 8)) & 0xff;
    hash[i + 24] = (ctx.state[6] >> (24 - i * 8)) & 0xff;
    hash[i + 28] = (ctx.state[7] >> (24 - i * 8)) & 0xff;
  }
}

void sha256(const void *data, const size_t len, unsigned char out[32])
{
  SHA256Ctx ctx;
  sha256_init(ctx);
  sha256_update(ctx, static_cast<const unsigned char *>(data), len);
  sha256_final(ctx, out);
}

void hmac_sha256(const unsigned char *key,
                 const size_t key_len,
                 const unsigned char *msg,
                 const size_t msg_len,
                 unsigned char out[32])
{
  unsigned char kpad[64];
  unsigned char ikey[64];
  unsigned char okey[64];
  memset(kpad, 0, sizeof(kpad));
  if (key_len > 64) {
    sha256(key, key_len, kpad);
  }
  else {
    memcpy(kpad, key, key_len);
  }
  for (int i = 0; i < 64; i++) {
    ikey[i] = kpad[i] ^ 0x36;
    okey[i] = kpad[i] ^ 0x5c;
  }
  SHA256Ctx ctx;
  sha256_init(ctx);
  sha256_update(ctx, ikey, 64);
  sha256_update(ctx, msg, msg_len);
  unsigned char inner[32];
  sha256_final(ctx, inner);
  sha256_init(ctx);
  sha256_update(ctx, okey, 64);
  sha256_update(ctx, inner, 32);
  sha256_final(ctx, out);
  secure_zero(inner, sizeof(inner));
  secure_zero(kpad, sizeof(kpad));
  secure_zero(ikey, sizeof(ikey));
  secure_zero(okey, sizeof(okey));
}

void pbkdf2_hmac_sha256(const unsigned char *password,
                        const size_t password_len,
                        const unsigned char *salt,
                        const size_t salt_len,
                        const int iters,
                        unsigned char *out,
                        const size_t out_len)
{
  unsigned char u[32];
  unsigned char t[32];
  unsigned char salt_block[64];
  BLI_assert(salt_len + 4 <= sizeof(salt_block));
  memcpy(salt_block, salt, salt_len);

  size_t produced = 0;
  uint32_t block = 1;
  while (produced < out_len) {
    salt_block[salt_len] = (block >> 24) & 0xff;
    salt_block[salt_len + 1] = (block >> 16) & 0xff;
    salt_block[salt_len + 2] = (block >> 8) & 0xff;
    salt_block[salt_len + 3] = block & 0xff;
    hmac_sha256(password, password_len, salt_block, salt_len + 4, u);
    memcpy(t, u, 32);
    for (int i = 1; i < iters; i++) {
      hmac_sha256(password, password_len, u, 32, u);
      for (int j = 0; j < 32; j++) {
        t[j] ^= u[j];
      }
    }
    const size_t copy = std::min<size_t>(32, out_len - produced);
    memcpy(out + produced, t, copy);
    produced += copy;
    block++;
  }
  secure_zero(u, sizeof(u));
  secure_zero(t, sizeof(t));
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name ChaCha20
 * \{ */

void chacha20_quarter(uint32_t &a, uint32_t &b, uint32_t &c, uint32_t &d)
{
  a += b;
  d ^= a;
  d = (d << 16) | (d >> 16);
  c += d;
  b ^= c;
  b = (b << 12) | (b >> 20);
  a += b;
  d ^= a;
  d = (d << 8) | (d >> 24);
  c += d;
  b ^= c;
  b = (b << 7) | (b >> 25);
}

void chacha20_block(const uint32_t input[16], unsigned char output[64])
{
  uint32_t x[16];
  memcpy(x, input, sizeof(x));
  for (int i = 0; i < 10; i++) {
    chacha20_quarter(x[0], x[4], x[8], x[12]);
    chacha20_quarter(x[1], x[5], x[9], x[13]);
    chacha20_quarter(x[2], x[6], x[10], x[14]);
    chacha20_quarter(x[3], x[7], x[11], x[15]);
    chacha20_quarter(x[0], x[5], x[10], x[15]);
    chacha20_quarter(x[1], x[6], x[11], x[12]);
    chacha20_quarter(x[2], x[7], x[8], x[13]);
    chacha20_quarter(x[3], x[4], x[9], x[14]);
  }
  for (int i = 0; i < 16; i++) {
    x[i] += input[i];
  }
  for (int i = 0; i < 16; i++) {
    output[i * 4] = x[i] & 0xff;
    output[i * 4 + 1] = (x[i] >> 8) & 0xff;
    output[i * 4 + 2] = (x[i] >> 16) & 0xff;
    output[i * 4 + 3] = (x[i] >> 24) & 0xff;
  }
}

void chacha20_xor(const unsigned char key[32],
                  const unsigned char nonce[12],
                  const unsigned char *in,
                  unsigned char *out,
                  const size_t len)
{
  uint32_t state[16];
  state[0] = 0x61707865;
  state[1] = 0x3320646e;
  state[2] = 0x79622d32;
  state[3] = 0x6b206574;
  for (int i = 0; i < 8; i++) {
    state[4 + i] = uint32_t(key[i * 4]) | (uint32_t(key[i * 4 + 1]) << 8) |
                   (uint32_t(key[i * 4 + 2]) << 16) | (uint32_t(key[i * 4 + 3]) << 24);
  }
  state[12] = 0;
  for (int i = 0; i < 3; i++) {
    state[13 + i] = uint32_t(nonce[i * 4]) | (uint32_t(nonce[i * 4 + 1]) << 8) |
                    (uint32_t(nonce[i * 4 + 2]) << 16) | (uint32_t(nonce[i * 4 + 3]) << 24);
  }

  unsigned char block[64];
  size_t offset = 0;
  while (offset < len) {
    chacha20_block(state, block);
    state[12]++;
    const size_t n = std::min<size_t>(64, len - offset);
    for (size_t i = 0; i < n; i++) {
      out[offset + i] = in[offset + i] ^ block[i];
    }
    offset += n;
  }
  secure_zero(block, sizeof(block));
  secure_zero(state, sizeof(state));
}

/** \} */

void derive_key(const StringRef password,
                const unsigned char salt[kLockSaltLen],
                const int iters,
                unsigned char key[kLockKeyLen])
{
#ifdef _WIN32
  BCRYPT_ALG_HANDLE alg = nullptr;
  if (BCryptOpenAlgorithmProvider(
          &alg, BCRYPT_SHA256_ALGORITHM, nullptr, BCRYPT_ALG_HANDLE_HMAC_FLAG) == 0)
  {
    const NTSTATUS status = BCryptDeriveKeyPBKDF2(alg,
                                                  PUCHAR(const_cast<char *>(password.data())),
                                                  ULONG(password.size()),
                                                  PUCHAR(const_cast<unsigned char *>(salt)),
                                                  ULONG(kLockSaltLen),
                                                  ULONGLONG(std::max(iters, 1)),
                                                  key,
                                                  ULONG(kLockKeyLen),
                                                  0);
    BCryptCloseAlgorithmProvider(alg, 0);
    if (status == 0) {
      return;
    }
  }
#endif
  pbkdf2_hmac_sha256(reinterpret_cast<const unsigned char *>(password.data()),
                     password.size(),
                     salt,
                     kLockSaltLen,
                     iters,
                     key,
                     kLockKeyLen);
}

void store_key_hash(const unsigned char key[kLockKeyLen], char hash[40])
{
  unsigned char digest[32];
  sha256(key, kLockKeyLen, digest);
  memset(hash, 0, 40);
  memcpy(hash, digest, 32);
  secure_zero(digest, sizeof(digest));
}

bool key_hash_matches(const unsigned char key[kLockKeyLen], const char hash[40])
{
  unsigned char digest[32];
  sha256(key, kLockKeyLen, digest);
  const bool ok = secure_equal(digest, hash, 32);
  secure_zero(digest, sizeof(digest));
  return ok;
}

bool encrypt_buffer(const unsigned char key[kLockKeyLen],
                    const unsigned char nonce[kLockNonceLen],
                    const unsigned char *plain,
                    const size_t plain_len,
                    Vector<unsigned char> &r_out)
{
  r_out.resize(4 + plain_len + kLockTagLen);
  memcpy(r_out.data(), kLockMagic, 4);
  chacha20_xor(key, nonce, plain, r_out.data() + 4, plain_len);

  /* HMAC over magic || nonce || ciphertext. */
  Vector<unsigned char> mac_msg;
  mac_msg.resize(4 + kLockNonceLen + plain_len);
  memcpy(mac_msg.data(), kLockMagic, 4);
  memcpy(mac_msg.data() + 4, nonce, kLockNonceLen);
  memcpy(mac_msg.data() + 4 + kLockNonceLen, r_out.data() + 4, plain_len);
  hmac_sha256(key, kLockKeyLen, mac_msg.data(), mac_msg.size(), r_out.data() + 4 + plain_len);
  return true;
}

bool decrypt_buffer(const unsigned char key[kLockKeyLen],
                    const unsigned char nonce[kLockNonceLen],
                    const unsigned char *blob,
                    const size_t blob_len,
                    Vector<unsigned char> &r_plain)
{
  if (blob_len < 4 + kLockTagLen) {
    return false;
  }
  if (memcmp(blob, kLockMagic, 4) != 0) {
    return false;
  }
  const size_t ct_len = blob_len - 4 - kLockTagLen;
  Vector<unsigned char> mac_msg;
  mac_msg.resize(4 + kLockNonceLen + ct_len);
  memcpy(mac_msg.data(), kLockMagic, 4);
  memcpy(mac_msg.data() + 4, nonce, kLockNonceLen);
  memcpy(mac_msg.data() + 4 + kLockNonceLen, blob + 4, ct_len);
  unsigned char tag[kLockTagLen];
  hmac_sha256(key, kLockKeyLen, mac_msg.data(), mac_msg.size(), tag);
  if (!secure_equal(tag, blob + 4 + ct_len, kLockTagLen)) {
    secure_zero(tag, sizeof(tag));
    return false;
  }
  secure_zero(tag, sizeof(tag));
  r_plain.resize(ct_len);
  chacha20_xor(key, nonce, blob + 4, r_plain.data(), ct_len);
  return true;
}

void collect_nested_groups(bNodeTree &ntree, Set<bNodeTree *> &r_nested)
{
  Vector<bNodeTree *> stack;
  Set<bNodeTree *> visited;
  visited.add(&ntree);
  stack.append(&ntree);
  while (!stack.is_empty()) {
    bNodeTree *cur = stack.pop_last();
    for (bNode &node : cur->nodes) {
      if (node.id == nullptr || GS(node.id->name) != ID_NT) {
        continue;
      }
      bNodeTree *nested = reinterpret_cast<bNodeTree *>(node.id);
      if (nested->id.flag & ID_FLAG_EMBEDDED_DATA) {
        continue;
      }
      if (visited.add(nested)) {
        r_nested.add(nested);
        stack.append(nested);
      }
    }
  }
}

void secure_delete_file(const char *path)
{
  FILE *fp = BLI_fopen(path, "r+b");
  if (fp) {
    if (fseek(fp, 0, SEEK_END) == 0) {
      const long sz = ftell(fp);
      if (sz > 0 && fseek(fp, 0, SEEK_SET) == 0) {
        unsigned char zeros[4096] = {};
        long remaining = sz;
        while (remaining > 0) {
          const size_t chunk = size_t(std::min<long>(remaining, long(sizeof(zeros))));
          if (fwrite(zeros, 1, chunk, fp) != chunk) {
            break;
          }
          remaining -= long(chunk);
        }
        fflush(fp);
      }
    }
    fclose(fp);
  }
  BLI_delete(path, false, false);
}

bool encrypt_with_key(Main &bmain, bNodeTree &ntree, const unsigned char key[kLockKeyLen]);
void wipe_nodes(bNodeTree &ntree);

void seal_one(Main & /*bmain*/, bNodeTree &ntree)
{
  if (!ntree.runtime) {
    return;
  }
  /* Keep nodes in the tree so modifiers still evaluate after leaving the editor
   * and after reloading the file. The password only gates viewing. */
  ntree.runtime->lock_decrypt_users = 0;
}

void clear_one(bNodeTree &ntree)
{
  memset(ntree.lock_password_hash, 0, sizeof(ntree.lock_password_hash));
  memset(ntree.lock_salt, 0, sizeof(ntree.lock_salt));
  memset(ntree.lock_nonce, 0, sizeof(ntree.lock_nonce));
  ntree.lock_blob_size = 0;
  ntree.lock_kdf_iters = 0;
  if (ntree.lock_blob) {
    MEM_delete(ntree.lock_blob);
    ntree.lock_blob = nullptr;
  }
  ntree.lock_used_ids_num = 0;
  if (ntree.lock_used_ids) {
    MEM_delete(ntree.lock_used_ids);
    ntree.lock_used_ids = nullptr;
  }
  if (ntree.runtime) {
    ntree.runtime->lock_decrypted = false;
    ntree.runtime->lock_decrypt_users = 0;
    ntree.runtime->lock_view_granted = false;
    secure_zero(ntree.runtime->lock_key, sizeof(ntree.runtime->lock_key));
  }
}

constexpr char kPlainMagic[4] = {'N', 'T', 'R', '1'};
constexpr int16_t kIdRefNode = 0;
constexpr int16_t kIdRefSockIn = 1;
constexpr int16_t kIdRefSockOut = 2;

#pragma pack(push, 1)
struct IdRefPacked {
  int32_t node_identifier;
  int16_t where;
  int16_t id_code;
  char sock_identifier[64];
  char name[256];
};
#pragma pack(pop)

bool socket_holds_id(const eNodeSocketDatatype type)
{
  switch (type) {
    case SOCK_OBJECT:
    case SOCK_IMAGE:
    case SOCK_COLLECTION:
    case SOCK_TEXTURE:
    case SOCK_MATERIAL:
    case SOCK_FONT:
    case SOCK_SCENE:
    case SOCK_TEXT_ID:
    case SOCK_MASK:
    case SOCK_SOUND:
      return true;
    default:
      return false;
  }
}

ID *socket_id_get(const bNodeSocket &sock)
{
  if (!socket_holds_id(eNodeSocketDatatype(sock.type)) || sock.default_value == nullptr) {
    return nullptr;
  }
  return *reinterpret_cast<ID *const *>(sock.default_value);
}

void socket_id_set(bNodeSocket &sock, ID *id)
{
  if (!socket_holds_id(eNodeSocketDatatype(sock.type)) || sock.default_value == nullptr) {
    return;
  }
  *reinterpret_cast<ID **>(sock.default_value) = id;
}

void collect_id_refs(const bNodeTree &ntree, Vector<IdRefPacked> &r_refs)
{
  auto add_ref = [&](const int32_t node_identifier,
                     const int16_t where,
                     const char *sock_identifier,
                     const ID *id) {
    if (id == nullptr) {
      return;
    }
    IdRefPacked ref{};
    ref.node_identifier = node_identifier;
    ref.where = where;
    ref.id_code = GS(id->name);
    if (sock_identifier) {
      BLI_strncpy(ref.sock_identifier, sock_identifier, sizeof(ref.sock_identifier));
    }
    BLI_strncpy(ref.name, id->name + 2, sizeof(ref.name));
    r_refs.append(ref);
  };

  for (const bNode &node : ntree.nodes) {
    add_ref(node.identifier, kIdRefNode, "", node.id);
    for (const bNodeSocket &sock : node.inputs) {
      add_ref(node.identifier, kIdRefSockIn, sock.identifier, socket_id_get(sock));
    }
    for (const bNodeSocket &sock : node.outputs) {
      add_ref(node.identifier, kIdRefSockOut, sock.identifier, socket_id_get(sock));
    }
  }
}

void apply_id_refs(Main &bmain, bNodeTree &ntree, const Span<IdRefPacked> refs)
{
  for (const IdRefPacked &ref : refs) {
    if (ref.name[0] == '\0') {
      continue;
    }
    bNode *node = ntree.node_by_id(ref.node_identifier);
    if (node == nullptr) {
      continue;
    }
    ID *found = BKE_libblock_find_name(&bmain, short(ref.id_code), ref.name);
    if (found == nullptr) {
      continue;
    }
    if (ref.where == kIdRefNode) {
      node->id = found;
      continue;
    }
    ListBaseT<bNodeSocket> &socks = (ref.where == kIdRefSockIn) ? node->inputs : node->outputs;
    for (bNodeSocket &sock : socks) {
      if (STREQ(sock.identifier, ref.sock_identifier)) {
        socket_id_set(sock, found);
        break;
      }
    }
  }
}

void append_u32(Vector<unsigned char> &dst, const uint32_t value)
{
  const size_t i = dst.size();
  dst.resize(i + 4);
  memcpy(dst.data() + i, &value, 4);
}

bool read_u32(const unsigned char *data, const size_t size, size_t &offset, uint32_t &r_value)
{
  if (offset + 4 > size) {
    return false;
  }
  memcpy(&r_value, data + offset, 4);
  offset += 4;
  return true;
}

thread_local int g_lock_serialize_depth = 0;

struct LockSerializeGuard {
  LockSerializeGuard()
  {
    g_lock_serialize_depth++;
  }
  ~LockSerializeGuard()
  {
    g_lock_serialize_depth--;
  }
};

bool serialize_tree(Main &bmain, bNodeTree &ntree, Vector<unsigned char> &r_bytes)
{
  LockSerializeGuard serialize_guard;
  Vector<IdRefPacked> refs;
  collect_id_refs(ntree, refs);

  blendfile::PartialWriteContext ctx{bmain};
  using AddOps = blendfile::PartialWriteContext::IDAddOperations;
  /* Nested node groups must be copied (interface + id) so group-node sockets survive
   * mini-blend liblink. Other ID deps are cleared and restored via IdRefPacked; adding
   * objects/meshes here would bloat the ciphertext and previously crashed when localized
   * copies still pointed at G_MAIN. */
  auto dep_filter = [](LibraryIDLinkCallbackData *cb,
                       blendfile::PartialWriteContext::IDAddOptions /*options*/) -> AddOps {
    if (cb->id_pointer && *cb->id_pointer && GS((*cb->id_pointer)->name) == ID_NT) {
      if (((*cb->id_pointer)->flag & ID_FLAG_EMBEDDED_DATA) == 0) {
        return AddOps(AddOps::ADD_DEPENDENCIES | AddOps::MAKE_LOCAL | AddOps::SET_FAKE_USER);
      }
    }
    return AddOps(AddOps::CLEAR_DEPENDENCIES);
  };
  ID *copied = ctx.id_add(&ntree.id,
                          {AddOps(AddOps::SET_FAKE_USER | AddOps::MAKE_LOCAL |
                                  AddOps::CLEAR_DEPENDENCIES)},
                          dep_filter);
  if (!copied || GS(copied->name) != ID_NT) {
    return false;
  }
  bNodeTree *copy_tree = reinterpret_cast<bNodeTree *>(copied);
  copy_tree->id.tag &= ~ID_TAG_LOCALIZED;
  {
    Set<bNodeTree *> nested_copies;
    collect_nested_groups(*copy_tree, nested_copies);
    for (bNodeTree *child : nested_copies) {
      child->id.tag &= ~ID_TAG_LOCALIZED;
    }
  }
  /* Write plaintext nodes into the mini-blend (do not skip via lock flags). */
  if (copy_tree->lock_blob) {
    MEM_delete(copy_tree->lock_blob);
    copy_tree->lock_blob = nullptr;
  }
  copy_tree->lock_blob_size = 0;
  copy_tree->lock_kdf_iters = 0;
  copy_tree->lock_used_ids_num = 0;
  if (copy_tree->lock_used_ids) {
    MEM_delete(copy_tree->lock_used_ids);
    copy_tree->lock_used_ids = nullptr;
  }
  memset(copy_tree->lock_password_hash, 0, sizeof(copy_tree->lock_password_hash));
  memset(copy_tree->lock_salt, 0, sizeof(copy_tree->lock_salt));
  memset(copy_tree->lock_nonce, 0, sizeof(copy_tree->lock_nonce));
  if (copy_tree->runtime) {
    copy_tree->runtime->lock_decrypted = false;
    copy_tree->runtime->lock_decrypt_users = 0;
    secure_zero(copy_tree->runtime->lock_key, sizeof(copy_tree->runtime->lock_key));
  }

  char dir[FILE_MAX];
  BLI_temp_directory_path_get(dir, sizeof(dir));
  BLI_path_slash_ensure(dir, sizeof(dir));
  char path[FILE_MAX];
  unsigned int rnd = 0;
  fill_random(&rnd, sizeof(rnd));
  SNPRINTF(path, "%sntree_lock_%u_%u.blend", dir, ntree.id.session_uid, rnd);

  ReportList reports;
  BKE_reports_init(&reports, RPT_STORE);
  const bool wrote = ctx.write(path, 0, BLO_WRITE_PATH_REMAP_NONE, reports);
  BKE_reports_free(&reports);
  node_tree_set_type(*copy_tree);
  if (!wrote) {
    CLOG_ERROR(&LOG, "Failed to serialize locked node tree '%s'", ntree.id.name + 2);
    secure_delete_file(path);
    return false;
  }

  size_t size = 0;
  void *data = BLI_file_read_binary_as_mem(path, 0, &size);
  secure_delete_file(path);
  if (!data || size == 0) {
    MEM_delete_void(data);
    return false;
  }

  r_bytes.clear();
  r_bytes.extend(Span(reinterpret_cast<const unsigned char *>(kPlainMagic), 4));
  append_u32(r_bytes, uint32_t(size));
  append_u32(r_bytes, uint32_t(refs.size()));
  const size_t blend_off = r_bytes.size();
  r_bytes.resize(blend_off + size);
  memcpy(r_bytes.data() + blend_off, data, size);
  MEM_delete_void(data);
  if (!refs.is_empty()) {
    const size_t refs_off = r_bytes.size();
    r_bytes.resize(refs_off + refs.size() * sizeof(IdRefPacked));
    memcpy(r_bytes.data() + refs_off, refs.data(), refs.size() * sizeof(IdRefPacked));
  }
  return true;
}

void collect_used_ids(Main &bmain, bNodeTree &ntree)
{
  if (ntree.lock_used_ids) {
    MEM_delete(ntree.lock_used_ids);
    ntree.lock_used_ids = nullptr;
  }
  ntree.lock_used_ids_num = 0;

  VectorSet<ID *> ids;
  BKE_library_foreach_ID_link(
      &bmain,
      &ntree.id,
      [&](LibraryIDLinkCallbackData *cb) -> int {
        if (!cb->id_pointer || !*cb->id_pointer) {
          return IDWALK_RET_NOP;
        }
        if (cb->cb_flag & (IDWALK_CB_LOOPBACK | IDWALK_CB_EMBEDDED | IDWALK_CB_EMBEDDED_NOT_OWNING))
        {
          return IDWALK_RET_NOP;
        }
        ID *id = *cb->id_pointer;
        if (id == &ntree.id) {
          return IDWALK_RET_NOP;
        }
        ids.add(id);
        return IDWALK_RET_NOP;
      },
      nullptr,
      IDWALK_NOP);

  if (ids.is_empty()) {
    return;
  }
  ntree.lock_used_ids_num = int(ids.size());
  ntree.lock_used_ids = MEM_new_array<ID *>(size_t(ntree.lock_used_ids_num), __func__);
  int i = 0;
  for (ID *id : ids) {
    ntree.lock_used_ids[i++] = id;
  }
}

void wipe_nodes(bNodeTree &ntree)
{
  while (bNode *node = static_cast<bNode *>(ntree.nodes.first())) {
    node_remove_node(nullptr, ntree, *node, false, false);
  }
  MEM_SAFE_DELETE(ntree.nested_node_refs);
  ntree.nested_node_refs_num = 0;
  /* Drop lazy analysis caches: they point into the nodes/sockets just freed and are only
   * recomputed on demand (e.g. by #analyse_reference_lifetimes), so nothing else clears
   * them. A sealed tree must not keep dangling pointers -- copying it (e.g. as a nested
   * dependency while serializing another locked group) would crash otherwise. */
  ntree.runtime->reference_lifetimes_info.reset();
  ntree.runtime->structure_type_interface.reset();
  ntree.runtime->group_output_node = nullptr;
  BKE_ntree_update_tag_all(&ntree);
}

void remap_loaded_ids(Main &file_main, Main &dst_main, bNodeTree &loaded)
{
  BKE_library_foreach_ID_link(
      &file_main,
      &loaded.id,
      [&](LibraryIDLinkCallbackData *cb) -> int {
        if (!cb->id_pointer || !*cb->id_pointer) {
          return IDWALK_RET_NOP;
        }
        if (cb->cb_flag & (IDWALK_CB_LOOPBACK | IDWALK_CB_EMBEDDED)) {
          return IDWALK_RET_NOP;
        }
        ID *id = *cb->id_pointer;
        if (id == &loaded.id) {
          return IDWALK_RET_NOP;
        }
        ID *found = BKE_libblock_find_name(&dst_main, GS(id->name), id->name + 2);
        if (found) {
          *cb->id_pointer = found;
        }
        else {
          *cb->id_pointer = nullptr;
        }
        return IDWALK_RET_NOP;
      },
      nullptr,
      IDWALK_NOP);
}

bool copy_nodes_from_loaded(bNodeTree &dst, bNodeTree &src)
{
  wipe_nodes(dst);

  Map<const bNodeSocket *, bNodeSocket *> socket_map;
  dst.runtime->nodes_by_id.clear();
  dst.runtime->nodes_by_id.reserve(src.all_nodes().size());

  int i = 0;
  for (const bNode &src_node : src.nodes) {
    bNode *new_node = node_copy_with_mapping(&dst,
                                             src_node,
                                             LIB_ID_CREATE_NO_USER_REFCOUNT,
                                             src_node.name,
                                             src_node.identifier,
                                             socket_map,
                                             true);
    if (!new_node) {
      return false;
    }
    new_node->runtime->index_in_tree = i++;
  }

  for (bNode *node : dst.all_nodes()) {
    if (node->parent) {
      node->parent = dst.runtime->nodes_by_id.lookup_key_as(node->parent->identifier);
    }
  }

  /* Refresh sockets against the destination tree (and live nested groups) before
   * restoring links, so identifier lookup hits the sockets that will stay. */
  for (bNode *node : dst.all_nodes()) {
    node_declaration_ensure(dst, *node);
  }

  auto find_socket = [](bNode &node, const eNodeSocketInOut in_out, const char *ident) -> bNodeSocket * {
    if (ident == nullptr || ident[0] == '\0') {
      return nullptr;
    }
    ListBaseT<bNodeSocket> &socks = (in_out == SOCK_IN) ? node.inputs : node.outputs;
    for (bNodeSocket &sock : socks) {
      if (STREQ(sock.identifier, ident)) {
        return &sock;
      }
    }
    return nullptr;
  };

  dst.links.clear_no_delete();
  for (const bNodeLink &src_link : src.links) {
    if (!src_link.fromnode || !src_link.tonode || !src_link.fromsock || !src_link.tosock) {
      continue;
    }
    bNode *fromnode = dst.runtime->nodes_by_id.lookup_key_as(src_link.fromnode->identifier);
    bNode *tonode = dst.runtime->nodes_by_id.lookup_key_as(src_link.tonode->identifier);
    if (!fromnode || !tonode) {
      continue;
    }
    bNodeSocket *fromsock = find_socket(*fromnode, SOCK_OUT, src_link.fromsock->identifier);
    bNodeSocket *tosock = find_socket(*tonode, SOCK_IN, src_link.tosock->identifier);
    if (!fromsock || !tosock) {
      continue;
    }
    bNodeLink &dst_link = node_add_link(dst, *fromnode, *fromsock, *tonode, *tosock);
    dst_link.flag = src_link.flag;
    dst_link.multi_input_sort_id = src_link.multi_input_sort_id;
    dst_link.tosock->link = &dst_link;
  }

  MEM_SAFE_DELETE(dst.nested_node_refs);
  dst.nested_node_refs_num = src.nested_node_refs_num;
  if (src.nested_node_refs && src.nested_node_refs_num > 0) {
    dst.nested_node_refs = MEM_new_array<bNestedNodeRef>(size_t(src.nested_node_refs_num),
                                                         __func__);
    memcpy(dst.nested_node_refs,
           src.nested_node_refs,
           sizeof(bNestedNodeRef) * size_t(src.nested_node_refs_num));
  }

  BKE_ntree_update_tag_all(&dst);
  return true;
}

bool encrypt_with_key(Main &bmain, bNodeTree &ntree, const unsigned char key[kLockKeyLen])
{
  if (ntree.nodes.first() == nullptr && ntree.lock_blob_size > 0) {
    /* Already sealed. */
    return true;
  }
  Vector<unsigned char> plain;
  if (!serialize_tree(bmain, ntree, plain)) {
    return false;
  }
  unsigned char nonce[kLockNonceLen];
  if (!fill_random(nonce, kLockNonceLen)) {
    return false;
  }
  Vector<unsigned char> blob;
  if (!encrypt_buffer(key, nonce, plain.data(), plain.size(), blob)) {
    secure_zero(plain.data(), plain.size());
    return false;
  }
  secure_zero(plain.data(), plain.size());

  collect_used_ids(bmain, ntree);

  if (ntree.lock_blob) {
    MEM_delete(ntree.lock_blob);
    ntree.lock_blob = nullptr;
  }
  ntree.lock_blob_size = int(blob.size());
  ntree.lock_blob = MEM_new_array<char>(size_t(ntree.lock_blob_size), __func__);
  memcpy(ntree.lock_blob, blob.data(), size_t(ntree.lock_blob_size));
  memcpy(ntree.lock_nonce, nonce, kLockNonceLen);
  /* NOTE: the password-derived fields (lock_password_hash/lock_salt/lock_kdf_iters) are
   * NOT touched here. They belong to the separate view/edit gate and are managed by
   * #node_tree_lock_apply / #node_tree_clear_lock. The blob is always encrypted with the
   * embedded black-box key so decryption on load needs no password. */
  return true;
}

bool decrypt_with_key(Main &bmain, bNodeTree &ntree, const unsigned char key[kLockKeyLen])
{
  if (ntree.lock_blob == nullptr || ntree.lock_blob_size <= 0) {
    return ntree.nodes.first() != nullptr;
  }
  Vector<unsigned char> plain;
  if (!decrypt_buffer(key,
                      reinterpret_cast<const unsigned char *>(ntree.lock_nonce),
                      reinterpret_cast<const unsigned char *>(ntree.lock_blob),
                      size_t(ntree.lock_blob_size),
                      plain))
  {
    return false;
  }

  const unsigned char *plain_data = plain.data();
  size_t plain_size = plain.size();
  Vector<IdRefPacked> refs;
  const unsigned char *blend_ptr = plain_data;
  size_t blend_size = plain_size;

  if (plain_size >= 12 && memcmp(plain_data, kPlainMagic, 4) == 0) {
    size_t offset = 4;
    uint32_t blend_len = 0;
    uint32_t ref_count = 0;
    if (!read_u32(plain_data, plain_size, offset, blend_len) ||
        !read_u32(plain_data, plain_size, offset, ref_count))
    {
      secure_zero(plain.data(), plain.size());
      return false;
    }
    if (offset + blend_len > plain_size) {
      secure_zero(plain.data(), plain.size());
      return false;
    }
    blend_ptr = plain_data + offset;
    blend_size = blend_len;
    offset += blend_len;
    const size_t refs_bytes = size_t(ref_count) * sizeof(IdRefPacked);
    if (ref_count > 0) {
      if (offset + refs_bytes > plain_size) {
        secure_zero(plain.data(), plain.size());
        return false;
      }
      refs.resize(ref_count);
      memcpy(refs.data(), plain_data + offset, refs_bytes);
    }
  }

  BlendFileData *bfd = BLO_read_from_memory(
      blend_ptr, int(blend_size), BLO_READ_SKIP_USERDEF, nullptr);
  secure_zero(plain.data(), plain.size());
  if (!bfd || !bfd->main) {
    if (bfd) {
      BLO_blendfiledata_free(bfd);
    }
    return false;
  }

  bNodeTree *loaded = nullptr;
  for (bNodeTree &tree : bfd->main->nodetrees) {
    if (STREQ(tree.id.name + 2, ntree.id.name + 2)) {
      loaded = &tree;
      break;
    }
  }
  if (!loaded && bfd->main->nodetrees.first()) {
    loaded = static_cast<bNodeTree *>(bfd->main->nodetrees.first());
  }
  if (!loaded) {
    BLO_blendfiledata_free(bfd);
    return false;
  }

  node_tree_set_type(*loaded);
  remap_loaded_ids(*bfd->main, bmain, *loaded);
  const bool ok = copy_nodes_from_loaded(ntree, *loaded);
  BLO_blendfiledata_free(bfd);
  if (ok) {
    apply_id_refs(bmain, ntree, refs);
    node_tree_set_type(ntree);
    BKE_main_ensure_invariants(bmain, ntree.id);
  }
  return ok;
}

}  // namespace

bool node_tree_is_locked(const bNodeTree &ntree)
{
  return ntree.lock_blob_size > 0 || ntree.lock_kdf_iters > 0 ||
         ntree.lock_password_hash[0] != '\0';
}

bool node_tree_contents_sealed(const bNodeTree &ntree)
{
  if (!node_tree_is_locked(ntree)) {
    return false;
  }
  if (ntree.runtime && ntree.runtime->lock_decrypted) {
    return false;
  }
  return ntree.lock_blob_size > 0 && ntree.nodes.first() == nullptr;
}

bool node_tree_lock_blocks_view(const bNodeTree &ntree)
{
  if (!node_tree_is_locked(ntree)) {
    return false;
  }
  return !(ntree.runtime && ntree.runtime->lock_view_granted);
}

bool node_tree_lock_shares_key(const bNodeTree &a, const bNodeTree &b)
{
  if (!node_tree_is_locked(a) || !node_tree_is_locked(b)) {
    return false;
  }
  /* Nested groups copy the parent's salt and key hash. Independent groups get a
   * fresh salt even if the typed password is the same, so they must not match. */
  if (a.lock_kdf_iters > 0 && b.lock_kdf_iters > 0) {
    return memcmp(a.lock_salt, b.lock_salt, sizeof(a.lock_salt)) == 0 &&
           memcmp(a.lock_password_hash, b.lock_password_hash, 32) == 0;
  }
  if (a.lock_kdf_iters == 0 && b.lock_kdf_iters == 0) {
    return STREQ(a.lock_password_hash, b.lock_password_hash);
  }
  return false;
}

bool node_tree_password_matches(const bNodeTree &ntree, const StringRef password)
{
  if (!node_tree_is_locked(ntree)) {
    return true;
  }
  if (ntree.lock_kdf_iters > 0) {
    unsigned char key[kLockKeyLen];
    derive_key(password,
               reinterpret_cast<const unsigned char *>(ntree.lock_salt),
               ntree.lock_kdf_iters,
               key);
    const bool ok = key_hash_matches(key, ntree.lock_password_hash);
    secure_zero(key, sizeof(key));
    return ok;
  }
  /* Legacy MD5 verifier (hash-only lock, nodes still plaintext). */
  constexpr StringRefNull salt = "bNodeTree.lock.v1";
  std::string buffer;
  buffer.append(salt);
  buffer.append(password);
  char digest[16];
  char hex[33] = {};
  BLI_hash_md5_buffer(buffer.c_str(), buffer.size(), digest);
  BLI_hash_md5_to_hexdigest(digest, hex);
  return STREQ(ntree.lock_password_hash, hex);
}

static bool lock_apply_with_key(bNodeTree &ntree,
                                const unsigned char key[kLockKeyLen],
                                const unsigned char salt[kLockSaltLen],
                                Set<bNodeTree *> &visiting)
{
  if (!visiting.add(&ntree)) {
    return true;
  }
  if (node_tree_is_locked(ntree) && ntree.lock_kdf_iters > 0) {
    return true;
  }

  Set<bNodeTree *> nested;
  collect_nested_groups(ntree, nested);
  for (bNodeTree *child : nested) {
    if (!lock_apply_with_key(*child, key, salt, visiting)) {
      return false;
    }
  }

  if (!ntree.runtime) {
    return false;
  }
  memcpy(ntree.lock_salt, salt, kLockSaltLen);
  store_key_hash(key, ntree.lock_password_hash);
  ntree.lock_kdf_iters = kLockKdfIters;
  memcpy(ntree.runtime->lock_key, key, kLockKeyLen);
  ntree.runtime->lock_decrypted = true;
  /* The author obviously knows the password they just set. */
  ntree.runtime->lock_view_granted = true;
  return true;
}

bool node_tree_lock_apply(Main & /*bmain*/,
                          bNodeTree &ntree,
                          const StringRef password,
                          ReportList *reports)
{
  if (password.is_empty()) {
    if (reports) {
      BKE_report(reports, RPT_ERROR, "Password cannot be empty");
    }
    return false;
  }
  unsigned char salt[kLockSaltLen];
  if (!fill_random(salt, kLockSaltLen)) {
    if (reports) {
      BKE_report(reports, RPT_ERROR, "Failed to generate encryption salt");
    }
    return false;
  }
  unsigned char key[kLockKeyLen];
  derive_key(password, salt, kLockKdfIters, key);
  Set<bNodeTree *> visiting;
  const bool ok = lock_apply_with_key(ntree, key, salt, visiting);
  secure_zero(key, sizeof(key));
  if (!ok && reports) {
    BKE_report(reports, RPT_ERROR, "Failed to lock node group");
  }
  return ok;
}

static bool unlock_tree_with_key(Main &bmain,
                                 bNodeTree &ntree,
                                 const unsigned char *key,
                                 ReportList *reports,
                                 Set<bNodeTree *> &visiting)
{
  if (!visiting.add(&ntree)) {
    return true;
  }
  if (ntree.runtime && ntree.runtime->lock_decrypted && ntree.nodes.first()) {
    /* Already plaintext; still walk nested groups. */
  }
  else if (key && ntree.lock_blob_size > 0 && ntree.nodes.first() == nullptr) {
    /* Blobs are encrypted with the embedded black-box key. Fall back to the
     * password-derived key for files written by the very first lock implementation. */
    if (!decrypt_with_key(bmain, ntree, kBlackboxKey) &&
        !decrypt_with_key(bmain, ntree, key))
    {
      if (reports) {
        BKE_report(reports, RPT_ERROR, "Failed to decrypt node group");
      }
      return false;
    }
  }
  if (ntree.runtime && key) {
    memcpy(ntree.runtime->lock_key, key, kLockKeyLen);
    ntree.runtime->lock_decrypted = true;
    ntree.runtime->lock_view_granted = true;
  }

  Set<bNodeTree *> nested;
  collect_nested_groups(ntree, nested);
  for (bNodeTree *child : nested) {
    if (!node_tree_is_locked(*child)) {
      continue;
    }
    if (key && child->lock_kdf_iters > 0 &&
        !key_hash_matches(key, child->lock_password_hash))
    {
      continue;
    }
    if (!unlock_tree_with_key(bmain, *child, key, reports, visiting)) {
      return false;
    }
  }
  return true;
}

bool node_tree_lock_unlock_contents(Main &bmain,
                                    bNodeTree &ntree,
                                    const StringRef password,
                                    ReportList *reports)
{
  if (!node_tree_is_locked(ntree)) {
    Set<bNodeTree *> visiting;
    return unlock_tree_with_key(bmain, ntree, nullptr, reports, visiting);
  }

  /* Empty password is only allowed for an already-authenticated editor session. */
  if (password.is_empty()) {
    if (ntree.runtime && ntree.runtime->lock_decrypted && ntree.nodes.first()) {
      Set<bNodeTree *> visiting;
      return unlock_tree_with_key(bmain, ntree, ntree.runtime->lock_key, reports, visiting);
    }
    if (reports) {
      BKE_report(reports, RPT_ERROR, "Incorrect password");
    }
    return false;
  }

  unsigned char key[kLockKeyLen];
  if (ntree.lock_kdf_iters > 0) {
    derive_key(password,
               reinterpret_cast<const unsigned char *>(ntree.lock_salt),
               ntree.lock_kdf_iters,
               key);
    if (!key_hash_matches(key, ntree.lock_password_hash)) {
      secure_zero(key, sizeof(key));
      if (reports) {
        BKE_report(reports, RPT_ERROR, "Incorrect password");
      }
      return false;
    }
  }
  else if (!node_tree_password_matches(ntree, password)) {
    if (reports) {
      BKE_report(reports, RPT_ERROR, "Incorrect password");
    }
    return false;
  }
  else {
    /* Upgrade legacy MD5 lock: one KDF, no extra verify. */
    unsigned char salt[kLockSaltLen];
    if (!fill_random(salt, kLockSaltLen)) {
      if (reports) {
        BKE_report(reports, RPT_ERROR, "Failed to generate encryption salt");
      }
      return false;
    }
    memcpy(ntree.lock_salt, salt, kLockSaltLen);
    derive_key(password, salt, kLockKdfIters, key);
    store_key_hash(key, ntree.lock_password_hash);
    ntree.lock_kdf_iters = kLockKdfIters;
  }

  Set<bNodeTree *> visiting;
  const bool ok = unlock_tree_with_key(bmain, ntree, key, reports, visiting);
  if (ok && ntree.runtime) {
    memcpy(ntree.runtime->lock_key, key, kLockKeyLen);
    ntree.runtime->lock_decrypted = true;
    ntree.runtime->lock_view_granted = true;
  }
  secure_zero(key, sizeof(key));
  return ok;
}

void node_tree_lock_seal_all_for_write(Main &bmain)
{
  if (g_lock_serialize_depth > 0) {
    /* Re-entrant write of a lock mini-blend; never seal from inside. */
    return;
  }
  if (!bmain.is_global_main) {
    /* Only seal real file saves. Partial writes (copy/paste buffers, asset
     * extraction, lock mini-blends) operate on their own Main and keep plaintext. */
    return;
  }
  /* Two phases: first encrypt every locked group while all trees still have their nodes
   * (serializing a group copies its nested groups as dependencies, which requires them to
   * be intact), only then strip the nodes. */
  Vector<bNodeTree *> sealed_trees;
  for (bNodeTree &tree : bmain.nodetrees) {
    if (!node_tree_is_locked(tree)) {
      continue;
    }
    if (tree.nodes.first() == nullptr) {
      /* Already sealed (blob kept from a previous save). */
      continue;
    }
    if (!encrypt_with_key(bmain, tree, kBlackboxKey)) {
      CLOG_ERROR(&LOG,
                 "Failed to encrypt node group '%s'; saving it unencrypted",
                 tree.id.name + 2);
      continue;
    }
    sealed_trees.append(&tree);
  }
  for (bNodeTree *tree : sealed_trees) {
    wipe_nodes(*tree);
  }
}

void node_tree_lock_auto_decrypt_all(Main &bmain)
{
  for (bNodeTree &tree : bmain.nodetrees) {
    if (!node_tree_is_locked(tree)) {
      continue;
    }
    if (tree.nodes.first() != nullptr) {
      /* Plaintext nodes already present. */
      continue;
    }
    if (tree.lock_blob == nullptr || tree.lock_blob_size <= 0) {
      continue;
    }
    if (!decrypt_with_key(bmain, tree, kBlackboxKey)) {
      CLOG_ERROR(&LOG, "Failed to decrypt locked node group '%s'", tree.id.name + 2);
      continue;
    }
    /* Deliberately do NOT set runtime->lock_decrypted / lock_view_granted here:
     * automatic decryption only serves evaluation. Viewing and editing the internals
     * still requires the user password. */
  }
}

void node_tree_lock_restore_after_write(Main &bmain)
{
  if (g_lock_serialize_depth > 0 || !bmain.is_global_main) {
    return;
  }
  node_tree_lock_auto_decrypt_all(bmain);
}

void node_tree_lock_seal(Main &bmain, bNodeTree &ntree)
{
  Set<bNodeTree *> nested;
  collect_nested_groups(ntree, nested);
  for (bNodeTree *child : nested) {
    if (node_tree_is_locked(*child)) {
      seal_one(bmain, *child);
    }
  }
  seal_one(bmain, ntree);
}

void node_tree_lock_session_acquire(bNodeTree &ntree)
{
  auto acquire_one = [](bNodeTree &tree) {
    if (tree.runtime) {
      tree.runtime->lock_decrypt_users++;
      tree.runtime->lock_decrypted = true;
      tree.runtime->lock_view_granted = true;
    }
  };
  acquire_one(ntree);
  Set<bNodeTree *> nested;
  collect_nested_groups(ntree, nested);
  for (bNodeTree *child : nested) {
    if (node_tree_is_locked(*child)) {
      acquire_one(*child);
    }
  }
}

void node_tree_lock_session_release(Main & /*bmain*/, bNodeTree &ntree)
{
  /* Keep decrypted nodes in RAM so modifiers keep working after you leave the
   * group. The editor still requires the password (session uids). Disk writes
   * always strip plaintext. */
  Set<bNodeTree *> nested;
  collect_nested_groups(ntree, nested);

  auto release_one = [&](bNodeTree &tree) {
    if (!tree.runtime) {
      return;
    }
    if (tree.runtime->lock_decrypt_users > 0) {
      tree.runtime->lock_decrypt_users--;
    }
    /* Encrypt on save, not on every group exit. */
  };

  release_one(ntree);
  for (bNodeTree *child : nested) {
    if (node_tree_is_locked(*child)) {
      release_one(*child);
    }
  }
}

void node_tree_clear_lock(bNodeTree &ntree)
{
  Set<bNodeTree *> nested;
  collect_nested_groups(ntree, nested);
  clear_one(ntree);
  for (bNodeTree *child : nested) {
    if (child->runtime && child->runtime->lock_decrypted) {
      clear_one(*child);
    }
  }
}

}  // namespace blender::bke
