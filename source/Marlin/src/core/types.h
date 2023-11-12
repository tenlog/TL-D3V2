/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2020 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 */
#pragma once

#include <math.h>
#include <stddef.h>

#include "millis_t.h"
#include "../inc/MarlinConfigPre.h"

class __FlashStringHelper;
typedef const __FlashStringHelper *progmem_str;

typedef const __FlashStringHelper* FSTR_P;
#ifndef FPSTR
  #define FPSTR(S) (reinterpret_cast<FSTR_P>(S))
#endif
#define FTOP(S) (reinterpret_cast<const char*>(S))


//
// Conditional type assignment magic. For example...
//
// typename IF<(MYOPT==12), int, float>::type myvar;
//
template <bool, class L, class R>
struct IF { typedef R type; };
template <class L, class R>
struct IF<true, L, R> { typedef L type; };

#define NUM_AXIS_GANG(V...) GANG_N(NUM_AXES, V)
#define NUM_AXIS_CODE(V...) CODE_N(NUM_AXES, V)
#define NUM_AXIS_LIST(V...) LIST_N(NUM_AXES, V)
#define NUM_AXIS_LIST_1(V)  LIST_N_1(NUM_AXES, V)
#define NUM_AXIS_ARRAY(V...) { NUM_AXIS_LIST(V) }
#define NUM_AXIS_ARRAY_1(V)  { NUM_AXIS_LIST_1(V) }
#define NUM_AXIS_ARGS(T...) NUM_AXIS_LIST(T x, T y, T z, T i, T j, T k)
#define NUM_AXIS_ELEM(O)    NUM_AXIS_LIST(O.x, O.y, O.z, O.i, O.j, O.k)
#define NUM_AXIS_DEFS(T,V)  NUM_AXIS_LIST(T x=V, T y=V, T z=V, T i=V, T j=V, T k=V)

#define MAIN_AXIS_NAMES     NUM_AXIS_LIST(X, Y, Z, I, J, K)
#define MAIN_AXIS_MAP(F)    MAP(F, MAIN_AXIS_NAMES)
#define STR_AXES_MAIN       NUM_AXIS_GANG("X", "Y", "Z", STR_I, STR_J, STR_K)

#define LOGICAL_AXIS_GANG(E,V...) NUM_AXIS_GANG(V) GANG_ITEM_E(E)
#define LOGICAL_AXIS_CODE(E,V...) NUM_AXIS_CODE(V) CODE_ITEM_E(E)
#define LOGICAL_AXIS_LIST(E,V...) NUM_AXIS_LIST(V) LIST_ITEM_E(E)
#define LOGICAL_AXIS_LIST_1(V)    NUM_AXIS_LIST_1(V) LIST_ITEM_E(V)
#define LOGICAL_AXIS_ARRAY(E,V...) { LOGICAL_AXIS_LIST(E,V) }
#define LOGICAL_AXIS_ARRAY_1(V)    { LOGICAL_AXIS_LIST_1(V) }
#define LOGICAL_AXIS_ARGS(T...) LOGICAL_AXIS_LIST(T e, T x, T y, T z, T i, T j, T k)
#define LOGICAL_AXIS_ELEM(O)    LOGICAL_AXIS_LIST(O.e, O.x, O.y, O.z, O.i, O.j, O.k)
#define LOGICAL_AXIS_DECL(T,V)  LOGICAL_AXIS_LIST(T e=V, T x=V, T y=V, T z=V, T i=V, T j=V, T k=V)

#define LOGICAL_AXIS_NAMES      LOGICAL_AXIS_LIST(E, X, Y, Z, I, J, K)
#define LOGICAL_AXIS_MAP(F)     MAP(F, LOGICAL_AXIS_NAMES)

#define STR_AXES_LOGICAL LOGICAL_AXIS_GANG("E", "X", "Y", "Z", STR_I, STR_J, STR_K)

#define XYZ_GANG(V...) GANG_N(PRIMARY_LINEAR_AXES, V)
#define XYZ_CODE(V...) CODE_N(PRIMARY_LINEAR_AXES, V)

#define SECONDARY_AXIS_GANG(V...) GANG_N(SECONDARY_AXES, V)
#define SECONDARY_AXIS_CODE(V...) CODE_N(SECONDARY_AXES, V)

#if HAS_EXTRUDERS
  #define LIST_ITEM_E(N) , N
  #define CODE_ITEM_E(N) ; N
  #define GANG_ITEM_E(N) N
#else
  #define LIST_ITEM_E(N)
  #define CODE_ITEM_E(N)
  #define GANG_ITEM_E(N)
#endif

#define AXIS_COLLISION(L) (AXIS4_NAME == L || AXIS5_NAME == L || AXIS6_NAME == L)

// Helpers
#define _RECIP(N) ((N) ? 1.0f / static_cast<float>(N) : 0.0f)
#define _ABS(N) ((N) < 0 ? -(N) : (N))
#define _LS(N)  (N = (T)(uint32_t(N) << v))
#define _RS(N)  (N = (T)(uint32_t(N) >> v))
#define FI FORCE_INLINE


// Define types based on largest bit width stored value required
#define bits_t(W)   typename IF<((W)>   16), uint32_t, typename IF<((W)>  8), uint16_t, uint8_t>::type>::type
#define uvalue_t(V) typename IF<((V)>65535), uint32_t, typename IF<((V)>255), uint16_t, uint8_t>::type>::type
#define value_t(V)  typename IF<((V)>32767),  int32_t, typename IF<((V)>127),  int16_t,  int8_t>::type>::type


// General Flags for some number of states
template<size_t N>
struct Flags {
  typedef value_t(N) flagbits_t;
  typedef struct { bool b0:1, b1:1, b2:1, b3:1, b4:1, b5:1, b6:1, b7:1; } N8;
  typedef struct { bool b0:1, b1:1, b2:1, b3:1, b4:1, b5:1, b6:1, b7:1, b8:1, b9:1, b10:1, b11:1, b12:1, b13:1, b14:1, b15:1; } N16;
  typedef struct { bool b0:1,  b1:1,  b2:1,  b3:1,  b4:1,  b5:1,  b6:1,  b7:1,  b8:1,  b9:1, b10:1, b11:1, b12:1, b13:1, b14:1, b15:1,
                       b16:1, b17:1, b18:1, b19:1, b20:1, b21:1, b22:1, b23:1, b24:1, b25:1, b26:1, b27:1, b28:1, b29:1, b30:1, b31:1; } N32;
  union {
    flagbits_t b;
    typename IF<(N>16), N32, typename IF<(N>8), N16, N8>::type>::type flag;
  };
  FI void reset()                            { b = 0; }
  FI void set(const int n, const bool onoff) { onoff ? set(n) : clear(n); }
  FI void set(const int n)                   { b |=  (flagbits_t)_BV(n); }
  FI void clear(const int n)                 { b &= ~(flagbits_t)_BV(n); }
  FI bool test(const int n) const            { return TEST(b, n); }
  FI bool operator[](const int n)            { return test(n); }
  FI bool operator[](const int n) const      { return test(n); }
  FI int size() const                        { return sizeof(b); }
  FI operator bool() const                   { return b; }
};

// Specialization for a single bool flag
template<>
struct Flags<1> {
  bool b;
  FI void reset()                            { b = false; }
  FI void set(const int n, const bool onoff) { onoff ? set(n) : clear(n); }
  FI void set(const int)                     { b = true; }
  FI void clear(const int)                   { b = false; }
  FI bool test(const int) const              { return b; }
  FI bool& operator[](const int)             { return b; }
  FI bool  operator[](const int) const       { return b; }
  FI int size() const                        { return sizeof(b); }
  FI operator bool() const                   { return b; }
};

typedef Flags<8> flags_8_t;
typedef Flags<16> flags_16_t;

// Flags for some axis states, with per-axis aliases xyzijkuvwe
typedef struct AxisFlags {
  union {
    struct Flags<LOGICAL_AXES> flags;
    struct { bool LOGICAL_AXIS_LIST(e:1, x:1, y:1, z:1, i:1, j:1, k:1, u:1, v:1, w:1); };
  };
  FI void reset()                            { flags.reset(); }
  FI void set(const int n)                   { flags.set(n); }
  FI void set(const int n, const bool onoff) { flags.set(n, onoff); }
  FI void clear(const int n)                 { flags.clear(n); }
  FI bool test(const int n) const            { return flags.test(n); }
  FI bool operator[](const int n)            { return flags[n]; }
  FI bool operator[](const int n) const      { return flags[n]; }
  FI int size() const                        { return sizeof(flags); }
  FI operator bool() const                   { return flags; }
} axis_flags_t;


//
// Enumerated axis indices
//
//  - X_AXIS, Y_AXIS, and Z_AXIS should be used for axes in Cartesian space
//  - A_AXIS, B_AXIS, and C_AXIS should be used for Steppers, corresponding to XYZ on Cartesians
//  - X_HEAD, Y_HEAD, and Z_HEAD should be used for Steppers on Core kinematics
//
enum AxisEnum : uint8_t {
  X_AXIS   = 0, A_AXIS = 0,
  Y_AXIS   = 1, B_AXIS = 1,
  Z_AXIS   = 2, C_AXIS = 2,
  E_AXIS   = 3,
  X_HEAD   = 4, Y_HEAD = 5, Z_HEAD = 6,
  E0_AXIS  = 3,
  E1_AXIS, E2_AXIS, E3_AXIS, E4_AXIS, E5_AXIS, E6_AXIS, E7_AXIS,
  ALL_AXES = 0xFE, NO_AXIS = 0xFF
};


//
// Loop over XYZE axes
//
#define LOOP_XYZ(VAR) LOOP_S_LE_N(VAR, X_AXIS, Z_AXIS)
#define LOOP_XYZE(VAR) LOOP_S_LE_N(VAR, X_AXIS, E_AXIS)
#define LOOP_XYZE_N(VAR) LOOP_S_L_N(VAR, X_AXIS, XYZE_N)
#define LOOP_ABC(VAR) LOOP_S_LE_N(VAR, A_AXIS, C_AXIS)
#define LOOP_ABCE(VAR) LOOP_S_LE_N(VAR, A_AXIS, E_AXIS)
#define LOOP_ABCE_N(VAR) LOOP_S_L_N(VAR, A_AXIS, XYZE_N)

//
// feedRate_t is just a humble float
//
typedef float feedRate_t;

//
// celsius_t is the native unit of temperature. Signed to handle a disconnected thermistor value (-14).
// For more resolition (e.g., for a chocolate printer) this may later be changed to Celsius x 100
//
typedef int16_t celsius_t;
typedef float celsius_float_t;

//
// On AVR pointers are only 2 bytes so use 'const float &' for 'const float'
//
#ifdef __AVR__
  typedef const float & const_float_t;
#else
  typedef const float const_float_t;
#endif
typedef const_float_t const_feedRate_t;
typedef const_float_t const_celsius_float_t;

// Conversion macros
#define MMM_TO_MMS(MM_M) feedRate_t(static_cast<float>(MM_M) / 60.0f)
#define MMS_TO_MMM(MM_S) (static_cast<float>(MM_S) * 60.0f)

//
// Coordinates structures for XY, XYZ, XYZE...
//

// Forward declarations
template<typename T> struct XYval;
template<typename T> struct XYZval;
template<typename T> struct XYZEval;

typedef struct XYval<bool>          xy_bool_t;
typedef struct XYZval<bool>        xyz_bool_t;
typedef struct XYZEval<bool>      xyze_bool_t;

typedef struct XYval<char>          xy_char_t;
typedef struct XYZval<char>        xyz_char_t;
typedef struct XYZEval<char>      xyze_char_t;

typedef struct XYval<unsigned char>     xy_uchar_t;
typedef struct XYZval<unsigned char>   xyz_uchar_t;
typedef struct XYZEval<unsigned char> xyze_uchar_t;

typedef struct XYval<int8_t>        xy_int8_t;
typedef struct XYZval<int8_t>      xyz_int8_t;
typedef struct XYZEval<int8_t>    xyze_int8_t;

typedef struct XYval<uint8_t>      xy_uint8_t;
typedef struct XYZval<uint8_t>    xyz_uint8_t;
typedef struct XYZEval<uint8_t>  xyze_uint8_t;

typedef struct XYval<int16_t>        xy_int_t;
typedef struct XYZval<int16_t>      xyz_int_t;
typedef struct XYZEval<int16_t>    xyze_int_t;

typedef struct XYval<uint16_t>      xy_uint_t;
typedef struct XYZval<uint16_t>    xyz_uint_t;
typedef struct XYZEval<uint16_t>  xyze_uint_t;

typedef struct XYval<int32_t>       xy_long_t;
typedef struct XYZval<int32_t>     xyz_long_t;
typedef struct XYZEval<int32_t>   xyze_long_t;

typedef struct XYval<uint32_t>     xy_ulong_t;
typedef struct XYZval<uint32_t>   xyz_ulong_t;
typedef struct XYZEval<uint32_t> xyze_ulong_t;

typedef struct XYZval<volatile int32_t>   xyz_vlong_t;
typedef struct XYZEval<volatile int32_t> xyze_vlong_t;

typedef struct XYval<float>        xy_float_t;
typedef struct XYZval<float>      xyz_float_t;
typedef struct XYZEval<float>    xyze_float_t;

typedef struct XYval<feedRate_t>     xy_feedrate_t;
typedef struct XYZval<feedRate_t>   xyz_feedrate_t;
typedef struct XYZEval<feedRate_t> xyze_feedrate_t;

typedef xy_uint8_t xy_byte_t;
typedef xyz_uint8_t xyz_byte_t;
typedef xyze_uint8_t xyze_byte_t;

typedef xyz_long_t abc_long_t;
typedef xyze_long_t abce_long_t;
typedef xyz_ulong_t abc_ulong_t;
typedef xyze_ulong_t abce_ulong_t;

typedef xy_float_t xy_pos_t;
typedef xyz_float_t xyz_pos_t;
typedef xyze_float_t xyze_pos_t;

typedef xy_float_t ab_float_t;
typedef xyz_float_t abc_float_t;
typedef xyze_float_t abce_float_t;

typedef ab_float_t ab_pos_t;
typedef abc_float_t abc_pos_t;
typedef abce_float_t abce_pos_t;

// External conversion methods
void toLogical(xy_pos_t &raw);
void toLogical(xyz_pos_t &raw);
void toLogical(xyze_pos_t &raw);
void toNative(xy_pos_t &raw);
void toNative(xyz_pos_t &raw);
void toNative(xyze_pos_t &raw);

//
// XY coordinates, counters, etc.
//
template<typename T>
struct XYval {
  union {
    struct { T x, y; };
    struct { T a, b; };
    T pos[2];
  };
  FI void set(const T px)                               { x = px; }
  FI void set(const T px, const T py)                   { x = px; y = py; }
  FI void set(const T (&arr)[XY])                       { x = arr[0]; y = arr[1]; }
  FI void set(const T (&arr)[XYZ])                      { x = arr[0]; y = arr[1]; }
  FI void set(const T (&arr)[XYZE])                     { x = arr[0]; y = arr[1]; }
  #if XYZE_N > XYZE
    FI void set(const T (&arr)[XYZE_N])                 { x = arr[0]; y = arr[1]; }
  #endif
  FI void reset()                                       { x = y = 0; }
  FI T magnitude()                                const { return (T)sqrtf(x*x + y*y); }
  FI operator T* ()                                     { return pos; }
  FI operator bool()                                    { return x || y; }
  FI XYval<T>           copy()                    const { return *this; }
  FI XYval<T>            ABS()                    const { return {{{ T(_ABS(x)), T(_ABS(y)) }}}; }
  FI XYval<int16_t>    asInt()                          { return { int16_t(x), int16_t(y) }; }
  FI XYval<int16_t>    asInt()                    const { return { int16_t(x), int16_t(y) }; }
  FI XYval<int32_t>   asLong()                          { return { int32_t(x), int32_t(y) }; }
  FI XYval<int32_t>   asLong()                    const { return { int32_t(x), int32_t(y) }; }
  FI XYval<int32_t>   ROUNDL()                          { return { int32_t(LROUND(x)), int32_t(LROUND(y)) }; }
  FI XYval<int32_t>   ROUNDL()                    const { return { int32_t(LROUND(x)), int32_t(LROUND(y)) }; }
  FI XYval<float>    asFloat()                          { return { static_cast<float>(x), static_cast<float>(y) }; }
  FI XYval<float>    asFloat()                    const { return { static_cast<float>(x), static_cast<float>(y) }; }
  FI XYval<float> reciprocal()                    const { return {  _RECIP(x),  _RECIP(y) }; }
  FI XYval<float>  asLogical()                    const { XYval<float> o = asFloat(); toLogical(o); return o; }
  FI XYval<float>   asNative()                    const { XYval<float> o = asFloat(); toNative(o);  return o; }
  FI operator XYZval<T>()                               { return {{{ x, y }}}; }
  FI operator XYZval<T>()                         const { return { x, y }; }
  FI operator XYZEval<T>()                              { return { x, y }; }
  FI operator XYZEval<T>()                        const { return { x, y }; }
  FI       T&  operator[](const int i)                  { return pos[i]; }
  FI const T&  operator[](const int i)            const { return pos[i]; }
  FI XYval<T>& operator= (const T v)                    { set(v,    v   ); return *this; }
  FI XYval<T>& operator= (const XYZval<T>  &rs)         { set(rs.x, rs.y); return *this; }
  FI XYval<T>& operator= (const XYZEval<T> &rs)         { set(rs.x, rs.y); return *this; }
  FI XYval<T>  operator+ (const XYval<T>   &rs)   const { XYval<T> ls = *this; ls.x += rs.x; ls.y += rs.y; return ls; }
  FI XYval<T>  operator+ (const XYval<T>   &rs)         { XYval<T> ls = *this; ls.x += rs.x; ls.y += rs.y; return ls; }
  FI XYval<T>  operator- (const XYval<T>   &rs)   const { XYval<T> ls = *this; ls.x -= rs.x; ls.y -= rs.y; return ls; }
  FI XYval<T>  operator- (const XYval<T>   &rs)         { XYval<T> ls = *this; ls.x -= rs.x; ls.y -= rs.y; return ls; }
  FI XYval<T>  operator* (const XYval<T>   &rs)   const { XYval<T> ls = *this; ls.x *= rs.x; ls.y *= rs.y; return ls; }
  FI XYval<T>  operator* (const XYval<T>   &rs)         { XYval<T> ls = *this; ls.x *= rs.x; ls.y *= rs.y; return ls; }
  FI XYval<T>  operator/ (const XYval<T>   &rs)   const { XYval<T> ls = *this; ls.x /= rs.x; ls.y /= rs.y; return ls; }
  FI XYval<T>  operator/ (const XYval<T>   &rs)         { XYval<T> ls = *this; ls.x /= rs.x; ls.y /= rs.y; return ls; }
  FI XYval<T>  operator+ (const XYZval<T>  &rs)   const { XYval<T> ls = *this; ls.x += rs.x; ls.y += rs.y; return ls; }
  FI XYval<T>  operator+ (const XYZval<T>  &rs)         { XYval<T> ls = *this; ls.x += rs.x; ls.y += rs.y; return ls; }
  FI XYval<T>  operator- (const XYZval<T>  &rs)   const { XYval<T> ls = *this; ls.x -= rs.x; ls.y -= rs.y; return ls; }
  FI XYval<T>  operator- (const XYZval<T>  &rs)         { XYval<T> ls = *this; ls.x -= rs.x; ls.y -= rs.y; return ls; }
  FI XYval<T>  operator* (const XYZval<T>  &rs)   const { XYval<T> ls = *this; ls.x *= rs.x; ls.y *= rs.y; return ls; }
  FI XYval<T>  operator* (const XYZval<T>  &rs)         { XYval<T> ls = *this; ls.x *= rs.x; ls.y *= rs.y; return ls; }
  FI XYval<T>  operator/ (const XYZval<T>  &rs)   const { XYval<T> ls = *this; ls.x /= rs.x; ls.y /= rs.y; return ls; }
  FI XYval<T>  operator/ (const XYZval<T>  &rs)         { XYval<T> ls = *this; ls.x /= rs.x; ls.y /= rs.y; return ls; }
  FI XYval<T>  operator+ (const XYZEval<T> &rs)   const { XYval<T> ls = *this; ls.x += rs.x; ls.y += rs.y; return ls; }
  FI XYval<T>  operator+ (const XYZEval<T> &rs)         { XYval<T> ls = *this; ls.x += rs.x; ls.y += rs.y; return ls; }
  FI XYval<T>  operator- (const XYZEval<T> &rs)   const { XYval<T> ls = *this; ls.x -= rs.x; ls.y -= rs.y; return ls; }
  FI XYval<T>  operator- (const XYZEval<T> &rs)         { XYval<T> ls = *this; ls.x -= rs.x; ls.y -= rs.y; return ls; }
  FI XYval<T>  operator* (const XYZEval<T> &rs)   const { XYval<T> ls = *this; ls.x *= rs.x; ls.y *= rs.y; return ls; }
  FI XYval<T>  operator* (const XYZEval<T> &rs)         { XYval<T> ls = *this; ls.x *= rs.x; ls.y *= rs.y; return ls; }
  FI XYval<T>  operator/ (const XYZEval<T> &rs)   const { XYval<T> ls = *this; ls.x /= rs.x; ls.y /= rs.y; return ls; }
  FI XYval<T>  operator/ (const XYZEval<T> &rs)         { XYval<T> ls = *this; ls.x /= rs.x; ls.y /= rs.y; return ls; }
  FI XYval<T>  operator* (const float &v)         const { XYval<T> ls = *this; ls.x *= v;    ls.y *= v;    return ls; }
  FI XYval<T>  operator* (const float &v)               { XYval<T> ls = *this; ls.x *= v;    ls.y *= v;    return ls; }
  FI XYval<T>  operator* (const int &v)           const { XYval<T> ls = *this; ls.x *= v;    ls.y *= v;    return ls; }
  FI XYval<T>  operator* (const int &v)                 { XYval<T> ls = *this; ls.x *= v;    ls.y *= v;    return ls; }
  FI XYval<T>  operator/ (const float &v)         const { XYval<T> ls = *this; ls.x /= v;    ls.y /= v;    return ls; }
  FI XYval<T>  operator/ (const float &v)               { XYval<T> ls = *this; ls.x /= v;    ls.y /= v;    return ls; }
  FI XYval<T>  operator/ (const int &v)           const { XYval<T> ls = *this; ls.x /= v;    ls.y /= v;    return ls; }
  FI XYval<T>  operator/ (const int &v)                 { XYval<T> ls = *this; ls.x /= v;    ls.y /= v;    return ls; }
  FI XYval<T>  operator>>(const int &v)           const { XYval<T> ls = *this; _RS(ls.x);    _RS(ls.y);    return ls; }
  FI XYval<T>  operator>>(const int &v)                 { XYval<T> ls = *this; _RS(ls.x);    _RS(ls.y);    return ls; }
  FI XYval<T>  operator<<(const int &v)           const { XYval<T> ls = *this; _LS(ls.x);    _LS(ls.y);    return ls; }
  FI XYval<T>  operator<<(const int &v)                 { XYval<T> ls = *this; _LS(ls.x);    _LS(ls.y);    return ls; }
  FI XYval<T>& operator+=(const XYval<T>   &rs)         { x += rs.x; y += rs.y; return *this; }
  FI XYval<T>& operator-=(const XYval<T>   &rs)         { x -= rs.x; y -= rs.y; return *this; }
  FI XYval<T>& operator*=(const XYval<T>   &rs)         { x *= rs.x; y *= rs.y; return *this; }
  FI XYval<T>& operator+=(const XYZval<T>  &rs)         { x += rs.x; y += rs.y; return *this; }
  FI XYval<T>& operator-=(const XYZval<T>  &rs)         { x -= rs.x; y -= rs.y; return *this; }
  FI XYval<T>& operator*=(const XYZval<T>  &rs)         { x *= rs.x; y *= rs.y; return *this; }
  FI XYval<T>& operator+=(const XYZEval<T> &rs)         { x += rs.x; y += rs.y; return *this; }
  FI XYval<T>& operator-=(const XYZEval<T> &rs)         { x -= rs.x; y -= rs.y; return *this; }
  FI XYval<T>& operator*=(const XYZEval<T> &rs)         { x *= rs.x; y *= rs.y; return *this; }
  FI XYval<T>& operator*=(const float &v)               { x *= v;    y *= v;    return *this; }
  FI XYval<T>& operator*=(const int &v)                 { x *= v;    y *= v;    return *this; }
  FI XYval<T>& operator>>=(const int &v)                { _RS(x);    _RS(y);    return *this; }
  FI XYval<T>& operator<<=(const int &v)                { _LS(x);    _LS(y);    return *this; }
  FI bool      operator==(const XYval<T>   &rs)         { return x == rs.x && y == rs.y; }
  FI bool      operator==(const XYZval<T>  &rs)         { return x == rs.x && y == rs.y; }
  FI bool      operator==(const XYZEval<T> &rs)         { return x == rs.x && y == rs.y; }
  FI bool      operator==(const XYval<T>   &rs)   const { return x == rs.x && y == rs.y; }
  FI bool      operator==(const XYZval<T>  &rs)   const { return x == rs.x && y == rs.y; }
  FI bool      operator==(const XYZEval<T> &rs)   const { return x == rs.x && y == rs.y; }
  FI bool      operator!=(const XYval<T>   &rs)         { return !operator==(rs); }
  FI bool      operator!=(const XYZval<T>  &rs)         { return !operator==(rs); }
  FI bool      operator!=(const XYZEval<T> &rs)         { return !operator==(rs); }
  FI bool      operator!=(const XYval<T>   &rs)   const { return !operator==(rs); }
  FI bool      operator!=(const XYZval<T>  &rs)   const { return !operator==(rs); }
  FI bool      operator!=(const XYZEval<T> &rs)   const { return !operator==(rs); }
  FI XYval<T>       operator-()                         { XYval<T> o = *this; o.x = -x; o.y = -y; return o; }
  FI const XYval<T> operator-()                   const { XYval<T> o = *this; o.x = -x; o.y = -y; return o; }
};

//
// XYZ coordinates, counters, etc.
//
template<typename T>
struct XYZval {
  union {
    struct { T x, y, z; };
    struct { T a, b, c; };
    T pos[3];
  };
  FI void set(const T px)                              { x = px; }
  FI void set(const T px, const T py)                  { x = px; y = py; }
  FI void set(const T px, const T py, const T pz)      { x = px; y = py; z = pz; }
  FI void set(const XYval<T> pxy, const T pz)          { x = pxy.x; y = pxy.y; z = pz; }
  FI void set(const T (&arr)[XY])                      { x = arr[0]; y = arr[1]; }
  FI void set(const T (&arr)[XYZ])                     { x = arr[0]; y = arr[1]; z = arr[2]; }
  FI void set(const T (&arr)[XYZE])                    { x = arr[0]; y = arr[1]; z = arr[2]; }
  #if XYZE_N > XYZE
    FI void set(const T (&arr)[XYZE_N])                { x = arr[0]; y = arr[1]; z = arr[2]; }
  #endif
  FI void reset()                                      { x = y = z = 0; }
  FI T magnitude()                               const { return (T)sqrtf(x*x + y*y + z*z); }
  FI operator T* ()                                    { return pos; }
  FI operator bool()                                   { return z || x || y; }
  FI XYZval<T>          copy()                   const { XYZval<T> o = *this; return o; }
  FI XYZval<T>           ABS()                   const { return { T(_ABS(x)), T(_ABS(y)), T(_ABS(z)) }; }
  FI XYZval<int16_t>   asInt()                         { return { int16_t(x), int16_t(y), int16_t(z) }; }
  FI XYZval<int16_t>   asInt()                   const { return { int16_t(x), int16_t(y), int16_t(z) }; }
  FI XYZval<int32_t>  asLong()                         { return { int32_t(x), int32_t(y), int32_t(z) }; }
  FI XYZval<int32_t>  asLong()                   const { return { int32_t(x), int32_t(y), int32_t(z) }; }
  FI XYZval<int32_t>  ROUNDL()                         { return { int32_t(LROUND(x)), int32_t(LROUND(y)), int32_t(LROUND(z)) }; }
  FI XYZval<int32_t>  ROUNDL()                   const { return { int32_t(LROUND(x)), int32_t(LROUND(y)), int32_t(LROUND(z)) }; }
  FI XYZval<float>   asFloat()                         { return { static_cast<float>(x), static_cast<float>(y), static_cast<float>(z) }; }
  FI XYZval<float>   asFloat()                   const { return { static_cast<float>(x), static_cast<float>(y), static_cast<float>(z) }; }
  FI XYZval<float> reciprocal()                  const { return {  _RECIP(x),  _RECIP(y),  _RECIP(z) }; }
  FI XYZval<float> asLogical()                   const { XYZval<float> o = asFloat(); toLogical(o); return o; }
  FI XYZval<float>  asNative()                   const { XYZval<float> o = asFloat(); toNative(o);  return o; }
  FI operator       XYval<T>&()                        { return *(XYval<T>*)this; }
  FI operator const XYval<T>&()                  const { return *(const XYval<T>*)this; }
  FI operator       XYZEval<T>()                 const { return { x, y, z }; }
  FI       T&   operator[](const int i)                { return pos[i]; }
  FI const T&   operator[](const int i)          const { return pos[i]; }
  FI XYZval<T>& operator= (const T v)                  { set(v,    v,    v   ); return *this; }
  FI XYZval<T>& operator= (const XYval<T>   &rs)       { set(rs.x, rs.y      ); return *this; }
  FI XYZval<T>& operator= (const XYZEval<T> &rs)       { set(rs.x, rs.y, rs.z); return *this; }
  FI XYZval<T>  operator+ (const XYval<T>   &rs) const { XYZval<T> ls = *this; ls.x += rs.x; ls.y += rs.y;               return ls; }
  FI XYZval<T>  operator+ (const XYval<T>   &rs)       { XYZval<T> ls = *this; ls.x += rs.x; ls.y += rs.y;               return ls; }
  FI XYZval<T>  operator- (const XYval<T>   &rs) const { XYZval<T> ls = *this; ls.x -= rs.x; ls.y -= rs.y;               return ls; }
  FI XYZval<T>  operator- (const XYval<T>   &rs)       { XYZval<T> ls = *this; ls.x -= rs.x; ls.y -= rs.y;               return ls; }
  FI XYZval<T>  operator* (const XYval<T>   &rs) const { XYZval<T> ls = *this; ls.x *= rs.x; ls.y *= rs.y;               return ls; }
  FI XYZval<T>  operator* (const XYval<T>   &rs)       { XYZval<T> ls = *this; ls.x *= rs.x; ls.y *= rs.y;               return ls; }
  FI XYZval<T>  operator/ (const XYval<T>   &rs) const { XYZval<T> ls = *this; ls.x /= rs.x; ls.y /= rs.y;               return ls; }
  FI XYZval<T>  operator/ (const XYval<T>   &rs)       { XYZval<T> ls = *this; ls.x /= rs.x; ls.y /= rs.y;               return ls; }
  FI XYZval<T>  operator+ (const XYZval<T>  &rs) const { XYZval<T> ls = *this; ls.x += rs.x; ls.y += rs.y; ls.z += rs.z; return ls; }
  FI XYZval<T>  operator+ (const XYZval<T>  &rs)       { XYZval<T> ls = *this; ls.x += rs.x; ls.y += rs.y; ls.z += rs.z; return ls; }
  FI XYZval<T>  operator- (const XYZval<T>  &rs) const { XYZval<T> ls = *this; ls.x -= rs.x; ls.y -= rs.y; ls.z -= rs.z; return ls; }
  FI XYZval<T>  operator- (const XYZval<T>  &rs)       { XYZval<T> ls = *this; ls.x -= rs.x; ls.y -= rs.y; ls.z -= rs.z; return ls; }
  FI XYZval<T>  operator* (const XYZval<T>  &rs) const { XYZval<T> ls = *this; ls.x *= rs.x; ls.y *= rs.y; ls.z *= rs.z; return ls; }
  FI XYZval<T>  operator* (const XYZval<T>  &rs)       { XYZval<T> ls = *this; ls.x *= rs.x; ls.y *= rs.y; ls.z *= rs.z; return ls; }
  FI XYZval<T>  operator/ (const XYZval<T>  &rs) const { XYZval<T> ls = *this; ls.x /= rs.x; ls.y /= rs.y; ls.z /= rs.z; return ls; }
  FI XYZval<T>  operator/ (const XYZval<T>  &rs)       { XYZval<T> ls = *this; ls.x /= rs.x; ls.y /= rs.y; ls.z /= rs.z; return ls; }
  FI XYZval<T>  operator+ (const XYZEval<T> &rs) const { XYZval<T> ls = *this; ls.x += rs.x; ls.y += rs.y; ls.z += rs.z; return ls; }
  FI XYZval<T>  operator+ (const XYZEval<T> &rs)       { XYZval<T> ls = *this; ls.x += rs.x; ls.y += rs.y; ls.z += rs.z; return ls; }
  FI XYZval<T>  operator- (const XYZEval<T> &rs) const { XYZval<T> ls = *this; ls.x -= rs.x; ls.y -= rs.y; ls.z -= rs.z; return ls; }
  FI XYZval<T>  operator- (const XYZEval<T> &rs)       { XYZval<T> ls = *this; ls.x -= rs.x; ls.y -= rs.y; ls.z -= rs.z; return ls; }
  FI XYZval<T>  operator* (const XYZEval<T> &rs) const { XYZval<T> ls = *this; ls.x *= rs.x; ls.y *= rs.y; ls.z *= rs.z; return ls; }
  FI XYZval<T>  operator* (const XYZEval<T> &rs)       { XYZval<T> ls = *this; ls.x *= rs.x; ls.y *= rs.y; ls.z *= rs.z; return ls; }
  FI XYZval<T>  operator/ (const XYZEval<T> &rs) const { XYZval<T> ls = *this; ls.x /= rs.x; ls.y /= rs.y; ls.z /= rs.z; return ls; }
  FI XYZval<T>  operator/ (const XYZEval<T> &rs)       { XYZval<T> ls = *this; ls.x /= rs.x; ls.y /= rs.y; ls.z /= rs.z; return ls; }
  FI XYZval<T>  operator* (const float &v)       const { XYZval<T> ls = *this; ls.x *= v;    ls.y *= v;    ls.z *= v;    return ls; }
  FI XYZval<T>  operator* (const float &v)             { XYZval<T> ls = *this; ls.x *= v;    ls.y *= v;    ls.z *= v;    return ls; }
  FI XYZval<T>  operator* (const int &v)         const { XYZval<T> ls = *this; ls.x *= v;    ls.y *= v;    ls.z *= v;    return ls; }
  FI XYZval<T>  operator* (const int &v)               { XYZval<T> ls = *this; ls.x *= v;    ls.y *= v;    ls.z *= v;    return ls; }
  FI XYZval<T>  operator/ (const float &v)       const { XYZval<T> ls = *this; ls.x /= v;    ls.y /= v;    ls.z /= v;    return ls; }
  FI XYZval<T>  operator/ (const float &v)             { XYZval<T> ls = *this; ls.x /= v;    ls.y /= v;    ls.z /= v;    return ls; }
  FI XYZval<T>  operator/ (const int &v)         const { XYZval<T> ls = *this; ls.x /= v;    ls.y /= v;    ls.z /= v;    return ls; }
  FI XYZval<T>  operator/ (const int &v)               { XYZval<T> ls = *this; ls.x /= v;    ls.y /= v;    ls.z /= v;    return ls; }
  FI XYZval<T>  operator>>(const int &v)         const { XYZval<T> ls = *this; _RS(ls.x); _RS(ls.y); _RS(ls.z); return ls; }
  FI XYZval<T>  operator>>(const int &v)               { XYZval<T> ls = *this; _RS(ls.x); _RS(ls.y); _RS(ls.z); return ls; }
  FI XYZval<T>  operator<<(const int &v)         const { XYZval<T> ls = *this; _LS(ls.x); _LS(ls.y); _LS(ls.z); return ls; }
  FI XYZval<T>  operator<<(const int &v)               { XYZval<T> ls = *this; _LS(ls.x); _LS(ls.y); _LS(ls.z); return ls; }
  FI XYZval<T>& operator+=(const XYval<T>   &rs)       { x += rs.x; y += rs.y;            return *this; }
  FI XYZval<T>& operator-=(const XYval<T>   &rs)       { x -= rs.x; y -= rs.y;            return *this; }
  FI XYZval<T>& operator*=(const XYval<T>   &rs)       { x *= rs.x; y *= rs.y;            return *this; }
  FI XYZval<T>& operator/=(const XYval<T>   &rs)       { x /= rs.x; y /= rs.y;            return *this; }
  FI XYZval<T>& operator+=(const XYZval<T>  &rs)       { x += rs.x; y += rs.y; z += rs.z; return *this; }
  FI XYZval<T>& operator-=(const XYZval<T>  &rs)       { x -= rs.x; y -= rs.y; z -= rs.z; return *this; }
  FI XYZval<T>& operator*=(const XYZval<T>  &rs)       { x *= rs.x; y *= rs.y; z *= rs.z; return *this; }
  FI XYZval<T>& operator/=(const XYZval<T>  &rs)       { x /= rs.x; y /= rs.y; z /= rs.z; return *this; }
  FI XYZval<T>& operator+=(const XYZEval<T> &rs)       { x += rs.x; y += rs.y; z += rs.z; return *this; }
  FI XYZval<T>& operator-=(const XYZEval<T> &rs)       { x -= rs.x; y -= rs.y; z -= rs.z; return *this; }
  FI XYZval<T>& operator*=(const XYZEval<T> &rs)       { x *= rs.x; y *= rs.y; z *= rs.z; return *this; }
  FI XYZval<T>& operator/=(const XYZEval<T> &rs)       { x /= rs.x; y /= rs.y; z /= rs.z; return *this; }
  FI XYZval<T>& operator*=(const float &v)             { x *= v;    y *= v;    z *= v;    return *this; }
  FI XYZval<T>& operator*=(const int &v)               { x *= v;    y *= v;    z *= v;    return *this; }
  FI XYZval<T>& operator>>=(const int &v)              { _RS(x);   _RS(y);   _RS(z);   return *this; }
  FI XYZval<T>& operator<<=(const int &v)              { _LS(x);   _LS(y);   _LS(z);   return *this; }
  FI bool       operator==(const XYZEval<T> &rs)       { return x == rs.x && y == rs.y && z == rs.z; }
  FI bool       operator!=(const XYZEval<T> &rs)       { return !operator==(rs); }
  FI bool       operator==(const XYZEval<T> &rs) const { return x == rs.x && y == rs.y && z == rs.z; }
  FI bool       operator!=(const XYZEval<T> &rs) const { return !operator==(rs); }
  FI XYZval<T>       operator-()                       { XYZval<T> o = *this; o.x = -x; o.y = -y; o.z = -z; return o; }
  FI const XYZval<T> operator-()                 const { XYZval<T> o = *this; o.x = -x; o.y = -y; o.z = -z; return o; }
};

//
// XYZE coordinates, counters, etc.
//
template<typename T>
struct XYZEval {
  union {
    struct{ T x, y, z, e; };
    struct{ T a, b, c; };
    T pos[4];
  };
  FI void reset()                                             { x = y = z = e = 0; }
  FI T magnitude()                                      const { return (T)sqrtf(x*x + y*y + z*z + e*e); }
  FI operator T* ()                                           { return pos; }
  FI operator bool()                                          { return e || z || x || y; }
  FI void set(const T px)                                     { x = px;                                        }
  FI void set(const T px, const T py)                         { x = px;     y = py;                            }
  FI void set(const T px, const T py, const T pz)             { x = px;     y = py;     z = pz;                }
  FI void set(const T px, const T py, const T pz, const T pe) { x = px;     y = py;     z = pz;     e = pe;    }
  FI void set(const XYval<T> pxy)                             { x = pxy.x;  y = pxy.y;                         }
  FI void set(const XYval<T> pxy, const T pz)                 { x = pxy.x;  y = pxy.y;  z = pz;                }
  FI void set(const XYZval<T> pxyz)                           { x = pxyz.x; y = pxyz.y; z = pxyz.z;            }
  FI void set(const XYval<T> pxy, const T pz, const T pe)     { x = pxy.x;  y = pxy.y;  z = pz;     e = pe;    }
  FI void set(const XYval<T> pxy, const XYval<T> pze)         { x = pxy.x;  y = pxy.y;  z = pze.z;  e = pze.e; }
  FI void set(const XYZval<T> pxyz, const T pe)               { x = pxyz.x; y = pxyz.y; z = pxyz.z; e = pe;    }
  FI void set(const T (&arr)[XY])                             { x = arr[0]; y = arr[1]; }
  FI void set(const T (&arr)[XYZ])                            { x = arr[0]; y = arr[1]; z = arr[2]; }
  FI void set(const T (&arr)[XYZE])                           { x = arr[0]; y = arr[1]; z = arr[2]; e = arr[3]; }
  #if XYZE_N > XYZE
    FI void set(const T (&arr)[XYZE_N])                       { x = arr[0]; y = arr[1]; z = arr[2]; e = arr[3]; }
  #endif
  FI XYZEval<T>          copy()                         const { return *this; }
  FI XYZEval<T>           ABS()                         const { return { T(_ABS(x)), T(_ABS(y)), T(_ABS(z)), T(_ABS(e)) }; }
  FI XYZEval<int16_t>   asInt()                               { return { int16_t(x), int16_t(y), int16_t(z), int16_t(e) }; }
  FI XYZEval<int16_t>   asInt()                         const { return { int16_t(x), int16_t(y), int16_t(z), int16_t(e) }; }
  FI XYZEval<int32_t>  asLong()                               { return { int32_t(x), int32_t(y), int32_t(z), int32_t(e) }; }
  FI XYZEval<int32_t>  asLong()                         const { return { int32_t(x), int32_t(y), int32_t(z), int32_t(e) }; }
  FI XYZEval<int32_t>  ROUNDL()                               { return { int32_t(LROUND(x)), int32_t(LROUND(y)), int32_t(LROUND(z)), int32_t(LROUND(e)) }; }
  FI XYZEval<int32_t>  ROUNDL()                         const { return { int32_t(LROUND(x)), int32_t(LROUND(y)), int32_t(LROUND(z)), int32_t(LROUND(e)) }; }
  FI XYZEval<float>   asFloat()                               { return { static_cast<float>(x), static_cast<float>(y), static_cast<float>(z), static_cast<float>(e) }; }
  FI XYZEval<float>   asFloat()                         const { return { static_cast<float>(x), static_cast<float>(y), static_cast<float>(z), static_cast<float>(e) }; }
  FI XYZEval<float> reciprocal()                        const { return {  _RECIP(x),  _RECIP(y),  _RECIP(z),  _RECIP(e) }; }
  FI XYZEval<float> asLogical()                         const { XYZEval<float> o = asFloat(); toLogical(o); return o; }
  FI XYZEval<float>  asNative()                         const { XYZEval<float> o = asFloat(); toNative(o);  return o; }
  FI operator       XYval<T>&()                               { return *(XYval<T>*)this; }
  FI operator const XYval<T>&()                         const { return *(const XYval<T>*)this; }
  FI operator       XYZval<T>&()                              { return *(XYZval<T>*)this; }
  FI operator const XYZval<T>&()                        const { return *(const XYZval<T>*)this; }
  FI       T&    operator[](const int i)                      { return pos[i]; }
  FI const T&    operator[](const int i)                const { return pos[i]; }
  FI XYZEval<T>& operator= (const T v)                        { set(v, v, v, v); return *this; }
  FI XYZEval<T>& operator= (const XYval<T>   &rs)             { set(rs.x, rs.y); return *this; }
  FI XYZEval<T>& operator= (const XYZval<T>  &rs)             { set(rs.x, rs.y, rs.z); return *this; }
  FI XYZEval<T>  operator+ (const XYval<T>   &rs)       const { XYZEval<T> ls = *this; ls.x += rs.x; ls.y += rs.y;                             return ls; }
  FI XYZEval<T>  operator+ (const XYval<T>   &rs)             { XYZEval<T> ls = *this; ls.x += rs.x; ls.y += rs.y;                             return ls; }
  FI XYZEval<T>  operator- (const XYval<T>   &rs)       const { XYZEval<T> ls = *this; ls.x -= rs.x; ls.y -= rs.y;                             return ls; }
  FI XYZEval<T>  operator- (const XYval<T>   &rs)             { XYZEval<T> ls = *this; ls.x -= rs.x; ls.y -= rs.y;                             return ls; }
  FI XYZEval<T>  operator* (const XYval<T>   &rs)       const { XYZEval<T> ls = *this; ls.x *= rs.x; ls.y *= rs.y;                             return ls; }
  FI XYZEval<T>  operator* (const XYval<T>   &rs)             { XYZEval<T> ls = *this; ls.x *= rs.x; ls.y *= rs.y;                             return ls; }
  FI XYZEval<T>  operator/ (const XYval<T>   &rs)       const { XYZEval<T> ls = *this; ls.x /= rs.x; ls.y /= rs.y;                             return ls; }
  FI XYZEval<T>  operator/ (const XYval<T>   &rs)             { XYZEval<T> ls = *this; ls.x /= rs.x; ls.y /= rs.y;                             return ls; }
  FI XYZEval<T>  operator+ (const XYZval<T>  &rs)       const { XYZEval<T> ls = *this; ls.x += rs.x; ls.y += rs.y; ls.z += rs.z;               return ls; }
  FI XYZEval<T>  operator+ (const XYZval<T>  &rs)             { XYZEval<T> ls = *this; ls.x += rs.x; ls.y += rs.y; ls.z += rs.z;               return ls; }
  FI XYZEval<T>  operator- (const XYZval<T>  &rs)       const { XYZEval<T> ls = *this; ls.x -= rs.x; ls.y -= rs.y; ls.z -= rs.z;               return ls; }
  FI XYZEval<T>  operator- (const XYZval<T>  &rs)             { XYZEval<T> ls = *this; ls.x -= rs.x; ls.y -= rs.y; ls.z -= rs.z;               return ls; }
  FI XYZEval<T>  operator* (const XYZval<T>  &rs)       const { XYZEval<T> ls = *this; ls.x *= rs.x; ls.y *= rs.y; ls.z *= rs.z;               return ls; }
  FI XYZEval<T>  operator* (const XYZval<T>  &rs)             { XYZEval<T> ls = *this; ls.x *= rs.x; ls.y *= rs.y; ls.z *= rs.z;               return ls; }
  FI XYZEval<T>  operator/ (const XYZval<T>  &rs)       const { XYZEval<T> ls = *this; ls.x /= rs.x; ls.y /= rs.y; ls.z /= rs.z;               return ls; }
  FI XYZEval<T>  operator/ (const XYZval<T>  &rs)             { XYZEval<T> ls = *this; ls.x /= rs.x; ls.y /= rs.y; ls.z /= rs.z;               return ls; }
  FI XYZEval<T>  operator+ (const XYZEval<T> &rs)       const { XYZEval<T> ls = *this; ls.x += rs.x; ls.y += rs.y; ls.z += rs.z; ls.e += rs.e; return ls; }
  FI XYZEval<T>  operator+ (const XYZEval<T> &rs)             { XYZEval<T> ls = *this; ls.x += rs.x; ls.y += rs.y; ls.z += rs.z; ls.e += rs.e; return ls; }
  FI XYZEval<T>  operator- (const XYZEval<T> &rs)       const { XYZEval<T> ls = *this; ls.x -= rs.x; ls.y -= rs.y; ls.z -= rs.z; ls.e -= rs.e; return ls; }
  FI XYZEval<T>  operator- (const XYZEval<T> &rs)             { XYZEval<T> ls = *this; ls.x -= rs.x; ls.y -= rs.y; ls.z -= rs.z; ls.e -= rs.e; return ls; }
  FI XYZEval<T>  operator* (const XYZEval<T> &rs)       const { XYZEval<T> ls = *this; ls.x *= rs.x; ls.y *= rs.y; ls.z *= rs.z; ls.e *= rs.e; return ls; }
  FI XYZEval<T>  operator* (const XYZEval<T> &rs)             { XYZEval<T> ls = *this; ls.x *= rs.x; ls.y *= rs.y; ls.z *= rs.z; ls.e *= rs.e; return ls; }
  FI XYZEval<T>  operator/ (const XYZEval<T> &rs)       const { XYZEval<T> ls = *this; ls.x /= rs.x; ls.y /= rs.y; ls.z /= rs.z; ls.e /= rs.e; return ls; }
  FI XYZEval<T>  operator/ (const XYZEval<T> &rs)             { XYZEval<T> ls = *this; ls.x /= rs.x; ls.y /= rs.y; ls.z /= rs.z; ls.e /= rs.e; return ls; }
  FI XYZEval<T>  operator* (const float &v)             const { XYZEval<T> ls = *this; ls.x *= v;    ls.y *= v;    ls.z *= v;    ls.e *= v;    return ls; }
  FI XYZEval<T>  operator* (const float &v)                   { XYZEval<T> ls = *this; ls.x *= v;    ls.y *= v;    ls.z *= v;    ls.e *= v;    return ls; }
  FI XYZEval<T>  operator* (const int &v)               const { XYZEval<T> ls = *this; ls.x *= v;    ls.y *= v;    ls.z *= v;    ls.e *= v;    return ls; }
  FI XYZEval<T>  operator* (const int &v)                     { XYZEval<T> ls = *this; ls.x *= v;    ls.y *= v;    ls.z *= v;    ls.e *= v;    return ls; }
  FI XYZEval<T>  operator/ (const float &v)             const { XYZEval<T> ls = *this; ls.x /= v;    ls.y /= v;    ls.z /= v;    ls.e /= v;    return ls; }
  FI XYZEval<T>  operator/ (const float &v)                   { XYZEval<T> ls = *this; ls.x /= v;    ls.y /= v;    ls.z /= v;    ls.e /= v;    return ls; }
  FI XYZEval<T>  operator/ (const int &v)               const { XYZEval<T> ls = *this; ls.x /= v;    ls.y /= v;    ls.z /= v;    ls.e /= v;    return ls; }
  FI XYZEval<T>  operator/ (const int &v)                     { XYZEval<T> ls = *this; ls.x /= v;    ls.y /= v;    ls.z /= v;    ls.e /= v;    return ls; }
  FI XYZEval<T>  operator>>(const int &v)               const { XYZEval<T> ls = *this; _RS(ls.x);    _RS(ls.y);    _RS(ls.z);    _RS(ls.e);    return ls; }
  FI XYZEval<T>  operator>>(const int &v)                     { XYZEval<T> ls = *this; _RS(ls.x);    _RS(ls.y);    _RS(ls.z);    _RS(ls.e);    return ls; }
  FI XYZEval<T>  operator<<(const int &v)               const { XYZEval<T> ls = *this; _LS(ls.x);    _LS(ls.y);    _LS(ls.z);    _LS(ls.e);    return ls; }
  FI XYZEval<T>  operator<<(const int &v)                     { XYZEval<T> ls = *this; _LS(ls.x);    _LS(ls.y);    _LS(ls.z);    _LS(ls.e);    return ls; }
  FI XYZEval<T>& operator+=(const XYval<T>   &rs)             { x += rs.x; y += rs.y;                       return *this; }
  FI XYZEval<T>& operator-=(const XYval<T>   &rs)             { x -= rs.x; y -= rs.y;                       return *this; }
  FI XYZEval<T>& operator*=(const XYval<T>   &rs)             { x *= rs.x; y *= rs.y;                       return *this; }
  FI XYZEval<T>& operator/=(const XYval<T>   &rs)             { x /= rs.x; y /= rs.y;                       return *this; }
  FI XYZEval<T>& operator+=(const XYZval<T>  &rs)             { x += rs.x; y += rs.y; z += rs.z;            return *this; }
  FI XYZEval<T>& operator-=(const XYZval<T>  &rs)             { x -= rs.x; y -= rs.y; z -= rs.z;            return *this; }
  FI XYZEval<T>& operator*=(const XYZval<T>  &rs)             { x *= rs.x; y *= rs.y; z *= rs.z;            return *this; }
  FI XYZEval<T>& operator/=(const XYZval<T>  &rs)             { x /= rs.x; y /= rs.y; z /= rs.z;            return *this; }
  FI XYZEval<T>& operator+=(const XYZEval<T> &rs)             { x += rs.x; y += rs.y; z += rs.z; e += rs.e; return *this; }
  FI XYZEval<T>& operator-=(const XYZEval<T> &rs)             { x -= rs.x; y -= rs.y; z -= rs.z; e -= rs.e; return *this; }
  FI XYZEval<T>& operator*=(const XYZEval<T> &rs)             { x *= rs.x; y *= rs.y; z *= rs.z; e *= rs.e; return *this; }
  FI XYZEval<T>& operator/=(const XYZEval<T> &rs)             { x /= rs.x; y /= rs.y; z /= rs.z; e /= rs.e; return *this; }
  FI XYZEval<T>& operator*=(const T &v)                       { x *= v;    y *= v;    z *= v;    e *= v;    return *this; }
  FI XYZEval<T>& operator>>=(const int &v)                    { _RS(x);    _RS(y);    _RS(z);    _RS(e);    return *this; }
  FI XYZEval<T>& operator<<=(const int &v)                    { _LS(x);    _LS(y);    _LS(z);    _LS(e);    return *this; }
  FI bool        operator==(const XYZval<T>  &rs)             { return x == rs.x && y == rs.y && z == rs.z; }
  FI bool        operator!=(const XYZval<T>  &rs)             { return !operator==(rs); }
  FI bool        operator==(const XYZval<T>  &rs)       const { return x == rs.x && y == rs.y && z == rs.z; }
  FI bool        operator!=(const XYZval<T>  &rs)       const { return !operator==(rs); }
  FI       XYZEval<T> operator-()                             { return { -x, -y, -z, -e }; }
  FI const XYZEval<T> operator-()                       const { return { -x, -y, -z, -e }; }
};

#undef _RECIP
#undef _ABS
#undef _LS
#undef _RS
#undef FI

const xyze_char_t axis_codes ={ 'X', 'Y', 'Z', 'E' };
#define XYZ_CHAR(A) ((char)('X' + A))
