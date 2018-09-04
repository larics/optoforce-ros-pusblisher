#ifdef OPTOFORCEAPI_EXPORTS
#ifndef __linux__
#define OPTOFORCE_API __declspec(dllexport)
#else
#define OPTOFORCE_API	__attribute__ ((visibility ("default")))
#endif
#else
#ifndef OPTOFORCE_STATIC
#ifndef __linux__
#define OPTOFORCE_API __declspec(dllimport)
#else
#define OPTOFORCE_API
#endif
#else
#define OPTOFORCE_API
#endif
#endif


OPTOFORCE_API const char * GetVersionString();
