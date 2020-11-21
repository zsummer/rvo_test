#pragma once

#ifndef COMM_DEF_H
#define COMM_DEF_H

#include <stdlib.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include <limits>
#include <string>
#include <vector>
#include <map>
#include <memory>
#include <list>
#include <array>
#include "fn_log.h"

typedef char s8;
typedef unsigned char u8;
typedef short int s16;
typedef unsigned short int u16;
typedef int s32;
typedef unsigned int u32;
typedef long long s64;
typedef unsigned long long u64;
typedef unsigned long pointer;
typedef unsigned int qq_t;
typedef int BOOL;
typedef float f32;

inline void empty_test(...) {}

#define error_tlog LOGFMTE
#define spell_error_tlog LOGFMTE
#define debug_tlog empty_test
#define spell_debug_tlog empty_test



enum SpellRangeType
{
    SPELL_RANGE_TYPE_NULL = 0, //无效
    SPELL_RANGE_TYPE_SINGLE = 1, //单体
    SPELL_RANGE_TYPE_AOE = 2, //群体AOE
    SPELL_RANGE_TYPE_INHERIT = 3, //继承
};


enum SpellRangeShape {
    SPELL_RANGE_SHAPE_CIRCLE = 0,
    SPELL_RANGE_SHAPE_FAN = 1,
    SPELL_RANGE_SHAPE_RECT = 2,
    SPELL_RANGE_SHAPE_RING = 3,
    SPELL_RANGE_SHAPE_FRAME = 4
};













#endif // COMM_DEF_H