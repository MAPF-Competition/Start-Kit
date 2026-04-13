/**
 * python_interface_test.cpp
 *
 * Comprehensive test suite for the Python interface.
 * Tests: binding types, zero-copy access, bridge classes, hybrid modes,
 *        end-to-end simulation, and large-scale data access performance.
 *
 * Uses pybind11 embedded interpreter and the MAPF module.
 * Loads real example problems for realistic testing.
 */
#include <cassert>
#include <iostream>
#include <fstream>
#include <chrono>
#include <vector>
#include <string>
#include <cmath>
#include <climits>

#include <pybind11/embed.h>
#include <pybind11/pybind11.h>
#include "opaque_types.h"

#include "SharedEnv.h"
#include "ActionModel.h"
#include "Plan.h"
#include "Tasks.h"
#include "States.h"
#include "Grid.h"
#include "Entry.h"
#include "Executor.h"
#include "CompetitionSystem.h"
#include "Logger.h"
#include "DelayGenerator.h"
#include "nlohmann/json.hpp"
#include "common.h"

#include "pyMAPFPlanner.hpp"
#include "pyTaskScheduler.hpp"
#include "pyExecutor.hpp"
#include "pyEntry.hpp"

namespace py = pybind11;
using json = nlohmann::json;

// ========================== Test helpers ==========================

static int tests_passed = 0;
static int tests_failed = 0;

#define TEST(name) \
    static void test_##name(); \
    struct TestReg_##name { \
        TestReg_##name() { \
            std::cout << "  [RUN ] " << #name << std::endl; \
            try { \
                test_##name(); \
                tests_passed++; \
                std::cout << "  [PASS] " << #name << std::endl; \
            } catch (const std::exception& e) { \
                tests_failed++; \
                std::cout << "  [FAIL] " << #name << ": " << e.what() << std::endl; \
            } catch (...) { \
                tests_failed++; \
                std::cout << "  [FAIL] " << #name << ": unknown exception" << std::endl; \
            } \
        } \
    }; \
    static void test_##name()

#define ASSERT_TRUE(cond) \
    if (!(cond)) throw std::runtime_error("Assertion failed: " #cond " at line " + std::to_string(__LINE__))
#define ASSERT_EQ(a, b) \
    if ((a) != (b)) throw std::runtime_error("Assertion failed: " #a " == " #b " (got " + std::to_string(a) + " vs " + std::to_string(b) + ") at line " + std::to_string(__LINE__))
#define ASSERT_GE(a, b) \
    if ((a) < (b)) throw std::runtime_error("Assertion failed: " #a " >= " #b " (got " + std::to_string(a) + " vs " + std::to_string(b) + ") at line " + std::to_string(__LINE__))
#define ASSERT_LE(a, b) \
    if ((a) > (b)) throw std::runtime_error("Assertion failed: " #a " <= " #b " (got " + std::to_string(a) + " vs " + std::to_string(b) + ") at line " + std::to_string(__LINE__))

// Helper: set up shared env with a real map
struct TestEnv {
    std::unique_ptr<Grid> grid;
    SharedEnvironment* env;
    std::vector<int> agents;
    std::vector<std::list<int>> tasks;
    int team_size;

    TestEnv(const std::string& json_path) : env(nullptr) {
        std::ifstream f(json_path);
        json data = json::parse(f);

        std::string base_folder;
        {
            size_t pos = json_path.find_last_of('/');
            if (pos != std::string::npos)
                base_folder = json_path.substr(0, pos + 1);
        }

        std::string map_path = data["mapFile"].get<std::string>();
        grid = std::make_unique<Grid>(base_folder + map_path);

        team_size = data["teamSize"].get<int>();
        agents = read_int_vec(base_folder + data["agentFile"].get<std::string>(), team_size);
        tasks = read_int_vec(base_folder + data["taskFile"].get<std::string>());

        env = new SharedEnvironment();
        env->num_of_agents = team_size;
        env->rows = grid->rows;
        env->cols = grid->cols;
        env->map = grid->map;
        env->map_name = map_path;
        env->curr_timestep = 0;
        env->system_timestep = 0;
        env->min_planner_communication_time = 1000;
        env->action_time = 100;
        env->max_counter = data.value("agentCounter", 10);

        // Initialize states
        env->curr_states.resize(team_size);
        env->system_states.resize(team_size);
        env->start_states.resize(team_size);
        for (int i = 0; i < team_size; i++) {
            State s(agents[i], 0, 0, env->max_counter);
            env->curr_states[i] = s;
            env->system_states[i] = s;
            env->start_states[i] = s;
        }

        // Initialize schedule and goal_locations
        env->curr_task_schedule.resize(team_size, -1);
        env->goal_locations.resize(team_size);
        env->staged_actions.resize(team_size);

        // Add first batch of tasks to task_pool
        int num_reveal = std::min((int)tasks.size(), (int)(team_size * 1.5));
        for (int i = 0; i < num_reveal; i++) {
            Task t;
            t.task_id = i;
            t.t_revealed = 0;
            for (auto loc : tasks[i]) {
                t.locations.push_back(loc);
            }
            env->task_pool[i] = t;
            env->new_tasks.push_back(i);
        }

        // Mark all agents as free
        for (int i = 0; i < team_size; i++) {
            env->new_freeagents.push_back(i);
        }
    }

    // Note: env is intentionally not deleted in the destructor.
    // Python bridge objects hold references to env's internal containers via
    // opaque zero-copy bindings. Deleting env while Python refs exist causes
    // use-after-free. In production, env outlives all components. In tests,
    // we accept the leak (env is small) for correctness.
    ~TestEnv() = default;
};


// =================== 1. BINDING TYPE TESTS ===================

void run_binding_tests() {
    std::cout << "\n=== Binding Type Tests ===" << std::endl;

    // Test that MAPF module is importable
    {
        std::cout << "  [RUN ] mapf_module_import" << std::endl;
        py::module_ mapf = py::module_::import("MAPF");
        ASSERT_TRUE(!mapf.is_none());
        tests_passed++;
        std::cout << "  [PASS] mapf_module_import" << std::endl;
    }

    // Test Action enum round-trip
    {
        std::cout << "  [RUN ] action_enum_roundtrip" << std::endl;
        py::module_ mapf = py::module_::import("MAPF");
        py::object fw = mapf.attr("Action").attr("FW");
        int val = fw.cast<int>();
        ASSERT_EQ(val, (int)Action::FW);
        ASSERT_EQ(val, 0);
        py::object w = mapf.attr("Action").attr("W");
        ASSERT_EQ(w.cast<int>(), 3);
        tests_passed++;
        std::cout << "  [PASS] action_enum_roundtrip" << std::endl;
    }

    // Test ExecutionCommand enum
    {
        std::cout << "  [RUN ] exec_cmd_enum" << std::endl;
        py::module_ mapf = py::module_::import("MAPF");
        ASSERT_EQ(mapf.attr("ExecutionCommand").attr("GO").cast<int>(), (int)ExecutionCommand::GO);
        ASSERT_EQ(mapf.attr("ExecutionCommand").attr("STOP").cast<int>(), (int)ExecutionCommand::STOP);
        tests_passed++;
        std::cout << "  [PASS] exec_cmd_enum" << std::endl;
    }

    // Test State construction and field access from Python
    {
        std::cout << "  [RUN ] state_roundtrip" << std::endl;
        py::module_ mapf = py::module_::import("MAPF");
        py::object state = mapf.attr("State")(42, 5, 2);
        ASSERT_EQ(state.attr("location").cast<int>(), 42);
        ASSERT_EQ(state.attr("timestep").cast<int>(), 5);
        ASSERT_EQ(state.attr("orientation").cast<int>(), 2);
        tests_passed++;
        std::cout << "  [PASS] state_roundtrip" << std::endl;
    }

    // Test Task field access from Python
    {
        std::cout << "  [RUN ] task_fields" << std::endl;
        Task t;
        t.task_id = 99;
        t.t_revealed = 5;
        t.locations = {10, 20, 30};
        t.idx_next_loc = 1;
        py::object py_task = py::cast(t);
        ASSERT_EQ(py_task.attr("task_id").cast<int>(), 99);
        ASSERT_EQ(py_task.attr("t_revealed").cast<int>(), 5);
        ASSERT_EQ(py_task.attr("idx_next_loc").cast<int>(), 1);
        // Check locations (opaque VectorInt)
        ASSERT_EQ(py::len(py_task.attr("locations")), 3);
        tests_passed++;
        std::cout << "  [PASS] task_fields" << std::endl;
    }
}


// =================== 2. ZERO-COPY TESTS ===================

void run_zero_copy_tests() {
    std::cout << "\n=== Zero-Copy Container Tests ===" << std::endl;

    // Test: VectorInt mutation from Python is visible in C++
    {
        std::cout << "  [RUN ] vector_int_zero_copy_mutation" << std::endl;
        std::vector<int> v = {1, 2, 3, 4, 5};
        py::object py_v = py::cast(v, py::return_value_policy::reference);
        // Modify from Python
        py_v.attr("append")(99);
        ASSERT_EQ(v.size(), (size_t)6);
        ASSERT_EQ(v[5], 99);
        // Read from Python
        ASSERT_EQ(py_v.attr("__getitem__")(0).cast<int>(), 1);
        ASSERT_EQ(py_v.attr("__getitem__")(5).cast<int>(), 99);
        tests_passed++;
        std::cout << "  [PASS] vector_int_zero_copy_mutation" << std::endl;
    }

    // Test: VectorState — modify C++ side, read from Python
    {
        std::cout << "  [RUN ] vector_state_cpp_to_python" << std::endl;
        std::vector<State> states;
        states.push_back(State(10, 0, 1));
        states.push_back(State(20, 0, 2));
        py::object py_states = py::cast(states, py::return_value_policy::reference);
        // Modify C++ side
        states[0].location = 999;
        // Python should see the change
        ASSERT_EQ(py_states.attr("__getitem__")(0).attr("location").cast<int>(), 999);
        tests_passed++;
        std::cout << "  [PASS] vector_state_cpp_to_python" << std::endl;
    }

    // Test: VectorVectorAction — nested access
    {
        std::cout << "  [RUN ] nested_vector_action_access" << std::endl;
        std::vector<std::vector<Action>> actions(3);
        actions[0] = {Action::FW, Action::CR};
        actions[1] = {Action::W};
        actions[2] = {};
        py::object py_actions = py::cast(actions, py::return_value_policy::reference);
        ASSERT_EQ(py::len(py_actions), 3);
        ASSERT_EQ(py::len(py_actions.attr("__getitem__")(0)), 2);
        ASSERT_EQ(py::len(py_actions.attr("__getitem__")(1)), 1);
        ASSERT_EQ(py::len(py_actions.attr("__getitem__")(2)), 0);
        // Append from Python
        py_actions.attr("__getitem__")(2).attr("append")(py::cast(Action::CCR));
        ASSERT_EQ(actions[2].size(), (size_t)1);
        ASSERT_EQ(actions[2][0], Action::CCR);
        tests_passed++;
        std::cout << "  [PASS] nested_vector_action_access" << std::endl;
    }

    // Test: TaskPool (unordered_map<int, Task>) access
    {
        std::cout << "  [RUN ] task_pool_access" << std::endl;
        std::unordered_map<int, Task> pool;
        Task t1; t1.task_id = 10; t1.locations = {100, 200};
        Task t2; t2.task_id = 20; t2.locations = {300};
        pool[10] = t1;
        pool[20] = t2;
        py::object py_pool = py::cast(pool, py::return_value_policy::reference);
        ASSERT_EQ(py::len(py_pool), 2);
        // Access by key
        py::object task = py_pool.attr("__getitem__")(10);
        ASSERT_EQ(task.attr("task_id").cast<int>(), 10);
        ASSERT_EQ(py::len(task.attr("locations")), 2);
        tests_passed++;
        std::cout << "  [PASS] task_pool_access" << std::endl;
    }

    // Test: SharedEnvironment fields accessible from Python without copy
    {
        std::cout << "  [RUN ] shared_env_reference" << std::endl;
        SharedEnvironment env;
        env.num_of_agents = 5;
        env.rows = 10;
        env.cols = 10;
        env.map.resize(100, 0);
        env.map[50] = 1;
        env.curr_states.resize(5);
        for (int i = 0; i < 5; i++) env.curr_states[i] = State(i, 0, 0);
        env.curr_task_schedule.resize(5, -1);
        env.staged_actions.resize(5);

        py::object py_env = py::cast(&env, py::return_value_policy::reference);
        ASSERT_EQ(py_env.attr("num_of_agents").cast<int>(), 5);
        ASSERT_EQ(py::len(py_env.attr("map")), 100);
        ASSERT_EQ(py_env.attr("map").attr("__getitem__")(50).cast<int>(), 1);

        // Mutate C++ side, check Python sees it
        env.curr_states[2].location = 777;
        ASSERT_EQ(py_env.attr("curr_states").attr("__getitem__")(2).attr("location").cast<int>(), 777);
        tests_passed++;
        std::cout << "  [PASS] shared_env_reference" << std::endl;
    }
}


// ============ 3. PYTHON BRIDGE COMPONENT TESTS ============

void run_bridge_tests() {
    std::cout << "\n=== Python Bridge Component Tests ===" << std::endl;

    // We need to add the test fixtures directory to Python path
    py::module_ sys = py::module_::import("sys");
    sys.attr("path").attr("insert")(0, "tests/python_test_fixtures");

    // ---- pyMAPFPlanner bridge ----
    {
        std::cout << "  [RUN ] py_planner_bridge" << std::endl;
        sys.attr("path").attr("insert")(0, "tests/python_test_fixtures");
        py::exec(R"(
import sys
import importlib
import dummy_planner
sys.modules['pyMAPFPlanner'] = dummy_planner
        )");

        TestEnv tenv("example_problems/random.domain/random_32_32_20_100.json");
        pyMAPFPlanner planner;
        planner.env = tenv.env;

        planner.initialize(1000);

        Plan plan;
        planner.plan(500, plan);
        ASSERT_EQ((int)plan.actions.size(), tenv.team_size);
        // All should be Wait actions
        for (int i = 0; i < tenv.team_size; i++) {
            ASSERT_EQ((int)plan.actions[i].size(), 1);
            ASSERT_EQ(plan.actions[i][0], Action::W);
        }
        tests_passed++;
        std::cout << "  [PASS] py_planner_bridge" << std::endl;
    }

    // ---- pyTaskScheduler bridge ----
    {
        std::cout << "  [RUN ] py_scheduler_bridge" << std::endl;
        py::exec(R"(
import sys
import dummy_scheduler
sys.modules['pyTaskScheduler'] = dummy_scheduler
        )");

        TestEnv tenv("example_problems/random.domain/random_32_32_20_100.json");
        pyTaskScheduler scheduler;
        scheduler.env = tenv.env;

        scheduler.initialize(1000);

        std::vector<int> schedule;
        scheduler.plan(500, schedule);
        ASSERT_EQ((int)schedule.size(), tenv.team_size);

        // The dummy scheduler should have assigned tasks to free agents
        int assigned = 0;
        for (int s : schedule) {
            if (s != -1) assigned++;
        }
        ASSERT_TRUE(assigned > 0);
        tests_passed++;
        std::cout << "  [PASS] py_scheduler_bridge" << std::endl;
    }

    // ---- pyExecutor bridge ----
    {
        std::cout << "  [RUN ] py_executor_bridge" << std::endl;
        py::exec(R"(
import sys
import dummy_executor
sys.modules['pyExecutor'] = dummy_executor
        )");

        TestEnv tenv("example_problems/random.domain/random_32_32_20_100.json");
        pyExecutor executor;
        executor.env = tenv.env;

        executor.initialize(1000);

        // Create a plan with some FW actions
        Plan plan;
        plan.actions.resize(tenv.team_size);
        for (int i = 0; i < tenv.team_size; i++) {
            plan.actions[i] = {Action::FW, Action::CR};
        }

        std::vector<std::vector<Action>> staged_actions(tenv.team_size);
        auto predicted = executor.process_new_plan(100, plan, staged_actions);

        ASSERT_EQ((int)predicted.size(), tenv.team_size);
        // staged_actions should have FW and CR appended (non-wait actions)
        for (int i = 0; i < tenv.team_size; i++) {
            ASSERT_EQ((int)staged_actions[i].size(), 2);
            ASSERT_EQ(staged_actions[i][0], Action::FW);
            ASSERT_EQ(staged_actions[i][1], Action::CR);
        }

        // Test next_command — should return GO since staged_actions are non-empty
        tenv.env->staged_actions = staged_actions;
        std::vector<ExecutionCommand> commands(tenv.team_size);
        executor.next_command(100, commands);
        ASSERT_EQ((int)commands.size(), tenv.team_size);
        for (auto cmd : commands) {
            ASSERT_EQ(cmd, ExecutionCommand::GO);
        }

        tests_passed++;
        std::cout << "  [PASS] py_executor_bridge" << std::endl;
    }
}


// ============ 4. HYBRID MODE TESTS ============

void run_hybrid_tests() {
    std::cout << "\n=== Hybrid Mode Tests (Python + C++ combinations) ===" << std::endl;

    // Test: pyEntry with only Python planner, C++ scheduler + executor
    {
        std::cout << "  [RUN ] hybrid_py_planner_cpp_rest" << std::endl;
        py::exec(R"(
import sys
import dummy_planner
sys.modules['pyMAPFPlanner'] = dummy_planner
        )");

        pyEntry entry(/*plannerPython=*/true, /*schedulerPython=*/false, /*executorPython=*/false);
        ASSERT_TRUE(entry.use_py_planner);
        ASSERT_TRUE(!entry.use_py_scheduler);
        ASSERT_TRUE(!entry.use_py_executor);

        Executor* exec = entry.get_executor();
        ASSERT_TRUE(exec != nullptr);
        // executor should be C++ default (not pyExecutor)
        ASSERT_TRUE(dynamic_cast<pyExecutor*>(exec) == nullptr);
        exec->env = nullptr; // prevent ~Executor from deleting shared env
        delete exec;

        tests_passed++;
        std::cout << "  [PASS] hybrid_py_planner_cpp_rest" << std::endl;
    }

    // Test: pyEntry with only Python scheduler
    {
        std::cout << "  [RUN ] hybrid_py_scheduler_cpp_rest" << std::endl;
        py::exec(R"(
import sys
import dummy_scheduler
sys.modules['pyTaskScheduler'] = dummy_scheduler
        )");

        pyEntry entry(/*plannerPython=*/false, /*schedulerPython=*/true, /*executorPython=*/false);
        ASSERT_TRUE(!entry.use_py_planner);
        ASSERT_TRUE(entry.use_py_scheduler);

        Executor* exec = entry.get_executor();
        ASSERT_TRUE(dynamic_cast<pyExecutor*>(exec) == nullptr);
        exec->env = nullptr;
        delete exec;

        tests_passed++;
        std::cout << "  [PASS] hybrid_py_scheduler_cpp_rest" << std::endl;
    }

    // Test: pyEntry with only Python executor
    {
        std::cout << "  [RUN ] hybrid_py_executor_cpp_rest" << std::endl;
        py::exec(R"(
import sys
import dummy_executor
sys.modules['pyExecutor'] = dummy_executor
        )");

        pyEntry entry(/*plannerPython=*/false, /*schedulerPython=*/false, /*executorPython=*/true);
        Executor* exec = entry.get_executor();
        ASSERT_TRUE(dynamic_cast<pyExecutor*>(exec) != nullptr);
        exec->env = nullptr;
        delete exec;

        tests_passed++;
        std::cout << "  [PASS] hybrid_py_executor_cpp_rest" << std::endl;
    }

    // Test: pyEntry with all Python
    {
        std::cout << "  [RUN ] hybrid_all_python" << std::endl;
        py::exec(R"(
import sys
import dummy_planner, dummy_scheduler, dummy_executor
sys.modules['pyMAPFPlanner'] = dummy_planner
sys.modules['pyTaskScheduler'] = dummy_scheduler
sys.modules['pyExecutor'] = dummy_executor
        )");

        pyEntry entry(true, true, true);
        Executor* exec = entry.get_executor();
        ASSERT_TRUE(dynamic_cast<pyExecutor*>(exec) != nullptr);
        exec->env = nullptr;
        delete exec;

        tests_passed++;
        std::cout << "  [PASS] hybrid_all_python" << std::endl;
    }

    // Test: all C++ (no Python)
    {
        std::cout << "  [RUN ] hybrid_all_cpp" << std::endl;
        pyEntry entry(false, false, false);
        Executor* exec = entry.get_executor();
        ASSERT_TRUE(dynamic_cast<pyExecutor*>(exec) == nullptr);
        exec->env = nullptr;
        delete exec;

        tests_passed++;
        std::cout << "  [PASS] hybrid_all_cpp" << std::endl;
    }
}


// ============ 5. END-TO-END SIMULATION TEST ============

void run_e2e_tests() {
    std::cout << "\n=== End-to-End Simulation Tests ===" << std::endl;

    // Test: C++ default — short simulation completes without crash
    {
        std::cout << "  [RUN ] e2e_cpp_default_short_sim" << std::endl;
        std::string input_file = "example_problems/random.domain/random_32_32_20_100.json";
        std::ifstream f(input_file);
        json data = json::parse(f);
        std::string base_folder = "example_problems/random.domain/";

        Grid grid(base_folder + data["mapFile"].get<std::string>());
        int team_size = data["teamSize"].get<int>();
        auto agents = read_int_vec(base_folder + data["agentFile"].get<std::string>(), team_size);
        auto tasks = read_int_vec(base_folder + data["taskFile"].get<std::string>());

        Entry* entry = new Entry();
        Executor* executor = new Executor(entry->env);
        float agent_size = data.value("agentSize", 1.0f);
        ActionModelWithRotate* model = new ActionModelWithRotate(grid, agent_size);
        Logger* logger = new Logger("", 5); // fatal only
        model->set_logger(logger);

        auto system_ptr = std::make_unique<BaseSystem>(
            grid, entry, executor, agents, tasks, model,
            data.value("agentCounter", 10));
        system_ptr->set_logger(logger);
        system_ptr->set_plan_time_limit(1000, 1000, 100, 100);
        system_ptr->set_preprocess_time_limit(5000);
        system_ptr->set_num_tasks_reveal(data.value("numTasksReveal", 1.0f));

        auto delay_config = parse_delay_config(data);
        system_ptr->set_delay_generator(std::make_unique<DelayGenerator>(delay_config, team_size));

        // Short simulation: 50 timesteps
        // Release GIL so worker threads can acquire it if needed
        {
            py::gil_scoped_release release;
            system_ptr->simulate(50, 50);
        }

        // If we get here without crash, basic C++ pipeline works
        system_ptr->saveResults("test_output_cpp.json", 3, false);
        // Note: original driver.cpp calls _exit() after saveResults to avoid
        // double-free issues in destructors. We do the same by leaking.
        system_ptr.release();
        std::remove("test_output_cpp.json");

        tests_passed++;
        std::cout << "  [PASS] e2e_cpp_default_short_sim" << std::endl;
    }

    // Test: Python planner only, short simulation
    {
        std::cout << "  [RUN ] e2e_py_planner_short_sim" << std::endl;
        py::exec(R"(
import sys
import dummy_planner
sys.modules['pyMAPFPlanner'] = dummy_planner
        )");

        std::string input_file = "example_problems/random.domain/random_32_32_20_100.json";
        std::ifstream f(input_file);
        json data = json::parse(f);
        std::string base_folder = "example_problems/random.domain/";

        Grid grid(base_folder + data["mapFile"].get<std::string>());
        int team_size = data["teamSize"].get<int>();
        auto agents = read_int_vec(base_folder + data["agentFile"].get<std::string>(), team_size);
        auto tasks = read_int_vec(base_folder + data["taskFile"].get<std::string>());

        // Python planner + C++ scheduler + C++ executor
        pyEntry* entry = new pyEntry(true, false, false);
        Executor* executor = entry->get_executor();

        float agent_size = data.value("agentSize", 1.0f);
        ActionModelWithRotate* model = new ActionModelWithRotate(grid, agent_size);
        Logger* logger = new Logger("", 5);
        model->set_logger(logger);

        auto system_ptr = std::make_unique<BaseSystem>(
            grid, entry, executor, agents, tasks, model,
            data.value("agentCounter", 10));
        system_ptr->set_logger(logger);
        system_ptr->set_plan_time_limit(1000, 1000, 100, 100);
        system_ptr->set_preprocess_time_limit(5000);
        system_ptr->set_num_tasks_reveal(data.value("numTasksReveal", 1.0f));

        auto delay_config = parse_delay_config(data);
        system_ptr->set_delay_generator(std::make_unique<DelayGenerator>(delay_config, team_size));

        // Short simulation with Python planner (returns all-wait)
        // Release GIL so worker threads in simulate() can acquire it for Python calls
        {
            py::gil_scoped_release release;
            system_ptr->simulate(50, 50);
        }
        system_ptr->saveResults("test_output_py_planner.json", 3, false);

        // Leak intentionally — same as driver.cpp using _exit()
        system_ptr.release();
        std::remove("test_output_py_planner.json");

        tests_passed++;
        std::cout << "  [PASS] e2e_py_planner_short_sim" << std::endl;
    }
}


// ============ 6. LARGE-SCALE PERFORMANCE TEST ============

void run_performance_tests() {
    std::cout << "\n=== Large-Scale Data Access Performance Tests (warehouse_large_5000) ===" << std::endl;

    std::string input_file = "example_problems/warehouse.domain/warehouse_large_5000.json";
    std::ifstream f(input_file);
    if (!f.is_open()) {
        std::cout << "  [SKIP] warehouse_large_5000.json not found, skipping perf tests" << std::endl;
        return;
    }
    json data = json::parse(f);
    std::string base_folder = "example_problems/warehouse.domain/";

    Grid grid(base_folder + data["mapFile"].get<std::string>());
    int team_size = data["teamSize"].get<int>();
    auto agents = read_int_vec(base_folder + data["agentFile"].get<std::string>(), team_size);
    auto tasks = read_int_vec(base_folder + data["taskFile"].get<std::string>());

    SharedEnvironment env;
    env.num_of_agents = team_size;
    env.rows = grid.rows;
    env.cols = grid.cols;
    env.map = grid.map;
    env.curr_timestep = 0;
    env.system_timestep = 0;
    env.min_planner_communication_time = 1000;
    env.action_time = 100;
    env.max_counter = data.value("agentCounter", 10);
    env.curr_states.resize(team_size);
    env.system_states.resize(team_size);
    env.start_states.resize(team_size);
    env.curr_task_schedule.resize(team_size, -1);
    env.goal_locations.resize(team_size);
    env.staged_actions.resize(team_size);

    for (int i = 0; i < team_size; i++) {
        State s(agents[i], 0, 0, env.max_counter);
        env.curr_states[i] = s;
        env.system_states[i] = s;
        env.start_states[i] = s;
    }

    // Create task pool
    int num_reveal = std::min((int)tasks.size(), (int)(team_size * 1.5));
    for (int i = 0; i < num_reveal; i++) {
        Task t;
        t.task_id = i;
        t.t_revealed = 0;
        for (auto loc : tasks[i]) t.locations.push_back(loc);
        env.task_pool[i] = t;
    }

    py::object py_env = py::cast(&env, py::return_value_policy::reference);

    // Test: map access (140*500 = 70,000 cells)
    {
        std::cout << "  [RUN ] perf_map_iterate_70k" << std::endl;
        int map_size = env.rows * env.cols;
        ASSERT_EQ(map_size, 140 * 500);
        ASSERT_EQ((int)env.map.size(), map_size);

        auto start = std::chrono::high_resolution_clock::now();
        // Iterate all cells from Python to measure access speed
        py::exec(R"(
total = 0
m = env.map
for i in range(len(m)):
    total += m[i]
        )", py::globals(), py::dict(py::arg("env") = py_env));
        auto end = std::chrono::high_resolution_clock::now();
        double ms = std::chrono::duration<double, std::milli>(end - start).count();
        std::cout << "    Map iteration (70k cells): " << ms << " ms" << std::endl;
        ASSERT_LE(ms, 500.0); // Should be well under 500ms for 70k ints
        tests_passed++;
        std::cout << "  [PASS] perf_map_iterate_70k" << std::endl;
    }

    // Test: iterate 5000 agent states from Python
    {
        std::cout << "  [RUN ] perf_states_iterate_5000" << std::endl;
        auto start = std::chrono::high_resolution_clock::now();
        py::exec(R"(
states = env.curr_states
locs = []
for i in range(len(states)):
    locs.append(states[i].location)
        )", py::globals(), py::dict(py::arg("env") = py_env));
        auto end = std::chrono::high_resolution_clock::now();
        double ms = std::chrono::duration<double, std::milli>(end - start).count();
        std::cout << "    States iteration (5000 agents): " << ms << " ms" << std::endl;
        ASSERT_LE(ms, 200.0); // Should be fast — no copy
        tests_passed++;
        std::cout << "  [PASS] perf_states_iterate_5000" << std::endl;
    }

    // Test: iterate task_pool (up to 7500 tasks)
    {
        std::cout << "  [RUN ] perf_task_pool_iterate" << std::endl;
        int pool_size = (int)env.task_pool.size();
        std::cout << "    Task pool size: " << pool_size << std::endl;

        auto start = std::chrono::high_resolution_clock::now();
        py::exec(R"(
pool = env.task_pool
count = 0
for k in pool:
    t = pool[k]
    count += len(t.locations)
        )", py::globals(), py::dict(py::arg("env") = py_env));
        auto end = std::chrono::high_resolution_clock::now();
        double ms = std::chrono::duration<double, std::milli>(end - start).count();
        std::cout << "    Task pool iteration (" << pool_size << " tasks): " << ms << " ms" << std::endl;
        ASSERT_LE(ms, 1000.0);
        tests_passed++;
        std::cout << "  [PASS] perf_task_pool_iterate" << std::endl;
    }

    // Test: iterate curr_task_schedule (5000 ints)
    {
        std::cout << "  [RUN ] perf_schedule_iterate_5000" << std::endl;
        auto start = std::chrono::high_resolution_clock::now();
        py::exec(R"(
sched = env.curr_task_schedule
total = 0
for i in range(len(sched)):
    total += sched[i]
        )", py::globals(), py::dict(py::arg("env") = py_env));
        auto end = std::chrono::high_resolution_clock::now();
        double ms = std::chrono::duration<double, std::milli>(end - start).count();
        std::cout << "    Schedule iteration (5000 agents): " << ms << " ms" << std::endl;
        ASSERT_LE(ms, 100.0);
        tests_passed++;
        std::cout << "  [PASS] perf_schedule_iterate_5000" << std::endl;
    }

    // Test: iterate staged_actions (5000 empty vectors — check overhead of nested access)
    {
        std::cout << "  [RUN ] perf_staged_actions_iterate_5000" << std::endl;
        auto start = std::chrono::high_resolution_clock::now();
        py::exec(R"(
sa = env.staged_actions
total = 0
for i in range(len(sa)):
    total += len(sa[i])
        )", py::globals(), py::dict(py::arg("env") = py_env));
        auto end = std::chrono::high_resolution_clock::now();
        double ms = std::chrono::duration<double, std::milli>(end - start).count();
        std::cout << "    Staged actions iteration (5000 agents): " << ms << " ms" << std::endl;
        ASSERT_LE(ms, 200.0);
        tests_passed++;
        std::cout << "  [PASS] perf_staged_actions_iterate_5000" << std::endl;
    }

    // Test: repeated access pattern — simulating what a planner does on each call
    {
        std::cout << "  [RUN ] perf_planner_access_pattern" << std::endl;
        // Typical planner: read all states, read map cells around each agent, read schedule
        auto start = std::chrono::high_resolution_clock::now();
        py::exec(R"(
states = env.curr_states
m = env.map
cols = env.cols
rows = env.rows
sched = env.curr_task_schedule

for iteration in range(10):  # simulate 10 plan calls
    for i in range(len(states)):
        loc = states[i].location
        ori = states[i].orientation
        # check current cell and adjacent
        if m[loc] == 0:
            r = loc // cols
            c = loc % cols
            # check 4 neighbors (if in bounds)
            if r > 0 and m[loc - cols] == 0:
                pass
            if r < rows - 1 and m[loc + cols] == 0:
                pass
            if c > 0 and m[loc - 1] == 0:
                pass
            if c < cols - 1 and m[loc + 1] == 0:
                pass
        )", py::globals(), py::dict(py::arg("env") = py_env));
        auto end = std::chrono::high_resolution_clock::now();
        double ms = std::chrono::duration<double, std::milli>(end - start).count();
        std::cout << "    Planner access pattern (10 iterations × 5000 agents): " << ms << " ms" << std::endl;
        ASSERT_LE(ms, 5000.0); // 10 full scans at 5000 agents
        tests_passed++;
        std::cout << "  [PASS] perf_planner_access_pattern" << std::endl;
    }
}


// ============ 7. EDGE CASE TESTS ============

void run_edge_case_tests() {
    std::cout << "\n=== Edge Case Tests ===" << std::endl;

    // Test: empty plan (0 actions per agent)
    {
        std::cout << "  [RUN ] edge_empty_plan" << std::endl;
        Plan plan;
        plan.actions.resize(3);
        // all empty
        py::object py_plan = py::cast(plan, py::return_value_policy::reference);
        ASSERT_EQ(py::len(py_plan.attr("actions")), 3);
        for (int i = 0; i < 3; i++) {
            ASSERT_EQ(py::len(py_plan.attr("actions").attr("__getitem__")(i)), 0);
        }
        tests_passed++;
        std::cout << "  [PASS] edge_empty_plan" << std::endl;
    }

    // Test: empty task pool
    {
        std::cout << "  [RUN ] edge_empty_task_pool" << std::endl;
        SharedEnvironment env;
        env.num_of_agents = 2;
        env.rows = 5;
        env.cols = 5;
        env.map.resize(25, 0);
        env.curr_states.resize(2);
        env.system_states.resize(2);
        env.curr_task_schedule.resize(2, -1);
        env.staged_actions.resize(2);
        env.goal_locations.resize(2);

        py::object py_env = py::cast(&env, py::return_value_policy::reference);
        py::exec(R"(
assert len(env.task_pool) == 0
assert len(env.new_tasks) == 0
        )", py::globals(), py::dict(py::arg("env") = py_env));  
        tests_passed++;
        std::cout << "  [PASS] edge_empty_task_pool" << std::endl;
    }

    // Test: Python modifies staged_actions in-place via nested access
    {
        std::cout << "  [RUN ] edge_inplace_staged_actions" << std::endl;
        SharedEnvironment env;
        env.num_of_agents = 2;
        env.rows = 5; env.cols = 5;
        env.map.resize(25, 0);
        env.curr_states.resize(2);
        env.system_states.resize(2);
        env.curr_task_schedule.resize(2, -1);
        env.staged_actions.resize(2);
        env.goal_locations.resize(2);

        py::object py_env = py::cast(&env, py::return_value_policy::reference);
        py::exec(R"(
import MAPF
env.staged_actions[0].append(MAPF.Action.FW)
env.staged_actions[0].append(MAPF.Action.CR)
env.staged_actions[1].append(MAPF.Action.CCR)
        )", py::globals(), py::dict(py::arg("env") = py_env));

        // Verify C++ side
        ASSERT_EQ((int)env.staged_actions[0].size(), 2);
        ASSERT_EQ(env.staged_actions[0][0], Action::FW);
        ASSERT_EQ(env.staged_actions[0][1], Action::CR);
        ASSERT_EQ((int)env.staged_actions[1].size(), 1);
        ASSERT_EQ(env.staged_actions[1][0], Action::CCR);

        tests_passed++;
        std::cout << "  [PASS] edge_inplace_staged_actions" << std::endl;
    }

    // Test: goal_locations nested pair access
    {
        std::cout << "  [RUN ] edge_goal_locations_pairs" << std::endl;
        SharedEnvironment env;
        env.num_of_agents = 2;
        env.rows = 5; env.cols = 5;
        env.map.resize(25, 0);
        env.curr_states.resize(2);
        env.system_states.resize(2);
        env.curr_task_schedule.resize(2, -1);
        env.staged_actions.resize(2);
        env.goal_locations.resize(2);
        env.goal_locations[0].push_back({10, 0});
        env.goal_locations[1].push_back({20, 5});
        env.goal_locations[1].push_back({15, 3});

        py::object py_env = py::cast(&env, py::return_value_policy::reference);
        py::exec(R"(
gl = env.goal_locations
assert len(gl[0]) == 1
import MAPF
assert MAPF.pair_first(gl[0][0]) == 10
assert MAPF.pair_second(gl[0][0]) == 0
assert len(gl[1]) == 2
assert MAPF.pair_first(gl[1][0]) == 20
assert MAPF.pair_first(gl[1][1]) == 15
assert MAPF.pair_second(gl[1][1]) == 3
        )", py::globals(), py::dict(py::arg("env") = py_env));
        tests_passed++;
        std::cout << "  [PASS] edge_goal_locations_pairs" << std::endl;
    }
}


// ========================== Main ==========================

int main(int argc, char** argv) {
    // Ensure output is flushed before any potential crash
    std::cout << std::unitbuf;
    std::cerr << std::unitbuf;

    py::scoped_interpreter guard{};

    // Add test fixtures and project directories to Python path
    py::module_ sys = py::module_::import("sys");
    sys.attr("path").attr("insert")(0, "tests/python_test_fixtures");
    sys.attr("path").attr("insert")(0, ".");   // for user python modules

    std::cout << "========================================" << std::endl;
    std::cout << "Python Interface Test Suite" << std::endl;
    std::cout << "========================================" << std::endl;

    run_binding_tests();
    run_zero_copy_tests();
    run_bridge_tests();
    run_hybrid_tests();
    run_e2e_tests();
    run_performance_tests();
    run_edge_case_tests();

    std::cout << "\n========================================" << std::endl;
    std::cout << "Results: " << tests_passed << " passed, " << tests_failed << " failed" << std::endl;
    std::cout << "========================================" << std::endl;

    // Use _exit to skip destructor cleanup — leaked objects (TestEnv, BaseSystem)
    // hold cross-references between C++ and Python that crash during teardown.
    // This matches driver.cpp's approach.
    _exit(tests_failed > 0 ? 1 : 0);
}
