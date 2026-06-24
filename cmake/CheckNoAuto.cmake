set(CHECK_NO_AUTO_SOURCE_DIR "${CMAKE_CURRENT_LIST_DIR}/.." CACHE PATH "Repository root to scan")

set(CHECK_NO_AUTO_PATTERNS
    "default_planner/*.cpp"
    "default_planner/*.h"
    "inc/*.h"
    "src/*.cpp"
    "src/*.h"
    "tests/*.cpp"
    "tests/*.h"
    "python/common/*.cpp"
    "python/common/*.h")

set(files)
foreach(pattern IN LISTS CHECK_NO_AUTO_PATTERNS)
    file(GLOB matched "${CHECK_NO_AUTO_SOURCE_DIR}/${pattern}")
    list(APPEND files ${matched})
endforeach()

set(found_auto FALSE)
foreach(file IN LISTS files)
    if(file MATCHES "/inc/nlohmann/")
        continue()
    endif()
    file(STRINGS "${file}" lines)
    set(line_number 0)
    foreach(line IN LISTS lines)
        math(EXPR line_number "${line_number} + 1")
        string(REGEX REPLACE "//.*$" "" code "${line}")
        if(code MATCHES "(^|[^A-Za-z0-9_])auto([ \\t\\*&]+|$)")
            file(RELATIVE_PATH rel "${CHECK_NO_AUTO_SOURCE_DIR}" "${file}")
            message(SEND_ERROR "${rel}:${line_number}: use explicit types instead of auto")
            set(found_auto TRUE)
        endif()
    endforeach()
endforeach()

if(found_auto)
    message(FATAL_ERROR "Found forbidden C++ auto keyword usage")
endif()
