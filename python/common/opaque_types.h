/**
 * opaque_types.h — Shared PYBIND11_MAKE_OPAQUE declarations.
 *
 * MUST be included in every translation unit that touches these types via pybind11.
 * This prevents pybind11 from auto-converting containers to Python lists (which copies data).
 * Instead, Python gets a thin wrapper referencing C++ memory directly (zero-copy).
 *
 * Include this INSTEAD of <pybind11/stl.h> in bridge files.
 */
#pragma once

#include <pybind11/pybind11.h>
#include <pybind11/stl_bind.h>
#include <vector>
#include <unordered_map>
#include <utility>

#include "States.h"
#include "ActionModel.h"
#include "Tasks.h"

PYBIND11_MAKE_OPAQUE(std::vector<int>)
PYBIND11_MAKE_OPAQUE(std::vector<State>)
PYBIND11_MAKE_OPAQUE(std::vector<Action>)
PYBIND11_MAKE_OPAQUE(std::vector<std::vector<Action>>)
PYBIND11_MAKE_OPAQUE(std::vector<std::pair<int, int>>)
PYBIND11_MAKE_OPAQUE(std::vector<std::vector<std::pair<int, int>>>)
PYBIND11_MAKE_OPAQUE(std::unordered_map<int, Task>)
