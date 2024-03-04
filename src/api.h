#pragma once

#if defined _WIN32 || defined __CYGWIN__
#  define OpenDoorRL_DLLIMPORT __declspec(dllimport)
#  define OpenDoorRL_DLLEXPORT __declspec(dllexport)
#  define OpenDoorRL_DLLLOCAL
#else
// On Linux, for GCC >= 4, tag symbols using GCC extension.
#  if __GNUC__ >= 4
#    define OpenDoorRL_DLLIMPORT __attribute__((visibility("default")))
#    define OpenDoorRL_DLLEXPORT __attribute__((visibility("default")))
#    define OpenDoorRL_DLLLOCAL __attribute__((visibility("hidden")))
#  else
// Otherwise (GCC < 4 or another compiler is used), export everything.
#    define OpenDoorRL_DLLIMPORT
#    define OpenDoorRL_DLLEXPORT
#    define OpenDoorRL_DLLLOCAL
#  endif // __GNUC__ >= 4
#endif // defined _WIN32 || defined __CYGWIN__

#ifdef OpenDoorRL_STATIC
// If one is using the library statically, get rid of
// extra information.
#  define OpenDoorRL_DLLAPI
#  define OpenDoorRL_LOCAL
#else
// Depending on whether one is building or using the
// library define DLLAPI to import or export.
#  ifdef OpenDoorRL_EXPORTS
#    define OpenDoorRL_DLLAPI OpenDoorRL_DLLEXPORT
#  else
#    define OpenDoorRL_DLLAPI OpenDoorRL_DLLIMPORT
#  endif // OpenDoorRL_EXPORTS
#  define OpenDoorRL_LOCAL OpenDoorRL_DLLLOCAL
#endif // OpenDoorRL_STATIC