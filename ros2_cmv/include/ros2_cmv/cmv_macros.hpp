#ifndef CMV_MACROS_HPP_
#define CMV_MACROS_HPP_

// Helper macro to convert a macro value to a string
#define STRINGIFY_HELPER(x) #x
#define STRINGIFY(x) STRINGIFY_HELPER(x)

// Macro to concatenate PROJECT_NAME with a header file
#define INCLUDE_PROJECT_HEADER(header) STRINGIFY(PROJECT_NAME/message_specific/MESSAGE_NAME/header)

#endif