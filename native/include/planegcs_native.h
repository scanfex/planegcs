/*
 * planegcs_native - a small C ABI around the FreeCAD PlaneGCS solver.
 *
 * The header is dependency free so it can be consumed from C, C++, C#/P/Invoke,
 * Rust, Python (ctypes), etc. All functions use the C calling convention.
 *
 * A context is NOT thread safe. Use one context per thread or protect it with
 * a lock. All indexes (parameters, geometry handles) are local to a context.
 */

#ifndef PLANEGCS_NATIVE_H
#define PLANEGCS_NATIVE_H

#if defined(_WIN32)
#  if defined(PLANEGCS_NATIVE_EXPORTS)
#    define PGCS_API __declspec(dllexport)
#  else
#    define PGCS_API __declspec(dllimport)
#  endif
#elif defined(__GNUC__)
#  define PGCS_API __attribute__((visibility("default")))
#else
#  define PGCS_API
#endif

#ifdef __cplusplus
extern "C" {
#endif

typedef struct pgcs_context pgcs_context;

/* Values match GCS::Algorithm. */
typedef enum pgcs_algorithm
{
    PGCS_ALGORITHM_BFGS = 0,
    PGCS_ALGORITHM_LEVENBERG_MARQUARDT = 1,
    PGCS_ALGORITHM_DOG_LEG = 2
} pgcs_algorithm;

typedef enum pgcs_result_code
{
    /* SolveStatus values returned by pgcs_solve. */
    PGCS_RESULT_SUCCESS = 0,
    PGCS_RESULT_CONVERGED = 1,
    PGCS_RESULT_FAILED = 2,
    PGCS_RESULT_SUCCESSFUL_SOLUTION_INVALID = 3,
    PGCS_RESULT_CANCELLED = 4,

    /* Error codes (any other API function). */
    PGCS_ERROR_INVALID_ARGUMENT = -1,
    PGCS_ERROR_UNKNOWN = -2,
    PGCS_ERROR_INVALID_HANDLE = -3,
    PGCS_ERROR_WRONG_GEOMETRY_TYPE = -4,
    PGCS_ERROR_GCS = -5,
    PGCS_ERROR_CANCELLED = -6
} pgcs_result_code;

/* Returns a static version string, for example "0.2.0". */
PGCS_API const char* pgcs_version(void);

/* Context lifecycle. */
PGCS_API pgcs_context* pgcs_create(void);
PGCS_API void pgcs_destroy(pgcs_context* ctx);

/* Removes all parameters, geometry and constraints from the context. */
PGCS_API void pgcs_clear(pgcs_context* ctx);

/* Removes all constraints with the given tag. */
PGCS_API int pgcs_clear_by_tag(pgcs_context* ctx, int tag);

/* Human readable description of the last error in this context. Never NULL. */
PGCS_API const char* pgcs_last_error(const pgcs_context* ctx);

/* Cooperative cancellation.
 *
 * pgcs_request_cancel() may be called from another thread while pgcs_solve(),
 * pgcs_diagnose() or pgcs_diagnose_full() is running: it only stores an atomic
 * flag, no other context state is touched. The running operation stops at the
 * next checkpoint and rolls the parameters of the interrupted subsystems back.
 * pgcs_clear_cancel() resets the flag; call it before starting a new operation.
 *
 * pgcs_solve() returns PGCS_RESULT_CANCELLED when interrupted, while
 * pgcs_diagnose()/pgcs_diagnose_full() return PGCS_ERROR_CANCELLED. */
PGCS_API void pgcs_request_cancel(pgcs_context* ctx);
PGCS_API void pgcs_clear_cancel(pgcs_context* ctx);
PGCS_API int pgcs_is_cancel_requested(const pgcs_context* ctx);

/* ------------------------------------------------------------------------- */
/* Parameters                                                               */
/* ------------------------------------------------------------------------- */

/* Adds a solver parameter and returns its index, or a negative error code. */
PGCS_API int pgcs_add_param(pgcs_context* ctx, double value, int is_fixed);

/* Returns the number of parameters or a negative error code. */
PGCS_API int pgcs_param_count(const pgcs_context* ctx);

PGCS_API int pgcs_get_param(const pgcs_context* ctx, int index, double* value);
PGCS_API int pgcs_set_param(pgcs_context* ctx, int index, double value, int is_fixed);
PGCS_API int pgcs_is_param_fixed(const pgcs_context* ctx, int index);

/* Solver settings. */
PGCS_API int pgcs_set_max_iterations(pgcs_context* ctx, int max_iterations);
PGCS_API int pgcs_get_max_iterations(const pgcs_context* ctx, int* max_iterations);
PGCS_API int pgcs_set_convergence_threshold(pgcs_context* ctx, double threshold);
PGCS_API int pgcs_get_convergence_threshold(const pgcs_context* ctx, double* threshold);
PGCS_API int pgcs_set_debug_mode(pgcs_context* ctx, int debug_mode);
PGCS_API int pgcs_get_debug_mode(const pgcs_context* ctx, int* debug_mode);

/* ------------------------------------------------------------------------- */
/* Geometry                                                                 */
/* ------------------------------------------------------------------------- */

/* Geometry handles are indexes into the context. Points must be created
 * before the lines/arcs that reference them. */
PGCS_API int pgcs_add_point(pgcs_context* ctx, int x_param, int y_param);
PGCS_API int pgcs_add_line(pgcs_context* ctx, int p1, int p2);
PGCS_API int pgcs_add_circle(pgcs_context* ctx, int center, int radius_param);

/* start/end are points, start_angle/end_angle/radius are parameter indexes.
 * Add pgcs_add_arc_rules() separately so the endpoints follow the angles. */
PGCS_API int pgcs_add_arc(pgcs_context* ctx,
                          int center,
                          int start,
                          int end,
                          int start_angle_param,
                          int end_angle_param,
                          int radius_param);

/* Copies geometry parameter values into `values`.
 * point:  x, y
 * line:   p1x, p1y, p2x, p2y
 * circle: cx, cy, radius
 * arc:    cx, cy, startx, starty, endx, endy, start_angle, end_angle, radius
 * Returns the number of values or a negative error code. */
PGCS_API int pgcs_get_geometry_params(const pgcs_context* ctx,
                                      int handle,
                                      double* values,
                                      int capacity);

/* ------------------------------------------------------------------------- */
/* Constraints                                                              */
/* ------------------------------------------------------------------------- */

/* Each function returns the supplied tag (>= 0) or a negative error code.
 * `driving` is 1 for driving constraints and 0 for measurements.
 * `scale` is the solver error scale, usually 1.0.
 * Tags must be unique within a context. */

PGCS_API int pgcs_add_coincident(pgcs_context* ctx,
                                 int p1,
                                 int p2,
                                 int tag,
                                 int driving,
                                 double scale);

PGCS_API int pgcs_add_p2p_distance(pgcs_context* ctx,
                                   int p1,
                                   int p2,
                                   int distance_param,
                                   int tag,
                                   int driving,
                                   double scale);

/* Signed point-to-line distance. `ccw` selects the side of the directed line
 * (`p1 -> p2`): 1 = positive/counter-clockwise side, 0 = opposite side. */
PGCS_API int pgcs_add_p2l_distance(pgcs_context* ctx,
                                   int point,
                                   int line,
                                   int distance_param,
                                   int ccw,
                                   int tag,
                                   int driving,
                                   double scale);

PGCS_API int pgcs_add_p2c_distance(pgcs_context* ctx,
                                   int point,
                                   int circle,
                                   int distance_param,
                                   int tag,
                                   int driving,
                                   double scale);

PGCS_API int pgcs_add_c2c_distance(pgcs_context* ctx,
                                   int circle1,
                                   int circle2,
                                   int distance_param,
                                   int tag,
                                   int driving,
                                   double scale);

/* Signed circle-to-line distance. `ccw` selects the side of the directed line
 * for the circle center. `internal` = 0 measures outside the circle
 * (radius + distance), 1 measures inside (radius - distance). */
PGCS_API int pgcs_add_c2l_distance(pgcs_context* ctx,
                                   int circle,
                                   int line,
                                   int distance_param,
                                   int ccw,
                                   int internal,
                                   int tag,
                                   int driving,
                                   double scale);

PGCS_API int pgcs_add_l2l_angle(pgcs_context* ctx,
                                int line1,
                                int line2,
                                int angle_param,
                                int tag,
                                int driving,
                                double scale);

PGCS_API int pgcs_add_parallel(pgcs_context* ctx,
                               int line1,
                               int line2,
                               int tag,
                               int driving,
                               double scale);

PGCS_API int pgcs_add_perpendicular(pgcs_context* ctx,
                                    int line1,
                                    int line2,
                                    int tag,
                                    int driving,
                                    double scale);

PGCS_API int pgcs_add_point_on_line(pgcs_context* ctx,
                                    int point,
                                    int line,
                                    int tag,
                                    int driving,
                                    double scale);

PGCS_API int pgcs_add_point_on_circle(pgcs_context* ctx,
                                      int point,
                                      int circle,
                                      int tag,
                                      int driving,
                                      double scale);

PGCS_API int pgcs_add_point_on_arc(pgcs_context* ctx,
                                   int point,
                                   int arc,
                                   int tag,
                                   int driving,
                                   double scale);

PGCS_API int pgcs_add_arc_rules(pgcs_context* ctx,
                                int arc,
                                int tag,
                                int driving,
                                double scale);

PGCS_API int pgcs_add_equal(pgcs_context* ctx,
                            int param1,
                            int param2,
                            int tag,
                            int driving,
                            double scale);

PGCS_API int pgcs_add_equal_length(pgcs_context* ctx,
                                   int line1,
                                   int line2,
                                   int tag,
                                   int driving,
                                   double scale);

PGCS_API int pgcs_add_equal_radius(pgcs_context* ctx,
                                   int circle1,
                                   int circle2,
                                   int tag,
                                   int driving,
                                   double scale);

PGCS_API int pgcs_add_circle_radius(pgcs_context* ctx,
                                    int circle,
                                    int radius_param,
                                    int tag,
                                    int driving,
                                    double scale);

PGCS_API int pgcs_add_arc_radius(pgcs_context* ctx,
                                 int arc,
                                 int radius_param,
                                 int tag,
                                 int driving,
                                 double scale);

PGCS_API int pgcs_add_circle_diameter(pgcs_context* ctx,
                                      int circle,
                                      int diameter_param,
                                      int tag,
                                      int driving,
                                      double scale);

PGCS_API int pgcs_add_arc_diameter(pgcs_context* ctx,
                                   int arc,
                                   int diameter_param,
                                   int tag,
                                   int driving,
                                   double scale);

PGCS_API int pgcs_add_difference(pgcs_context* ctx,
                                 int param1,
                                 int param2,
                                 int difference_param,
                                 int tag,
                                 int driving,
                                 double scale);

PGCS_API int pgcs_add_symmetric_line(pgcs_context* ctx,
                                     int p1,
                                     int p2,
                                     int line,
                                     int tag,
                                     int driving,
                                     double scale);

PGCS_API int pgcs_add_symmetric_point(pgcs_context* ctx,
                                      int p1,
                                      int p2,
                                      int point,
                                      int tag,
                                      int driving,
                                      double scale);

/* Line-circle/line-arc tangency. `ccw` selects the side of the directed line
 * on which the circle/arc center lies. */
PGCS_API int pgcs_add_tangent_line_circle(pgcs_context* ctx,
                                          int line,
                                          int circle,
                                          int ccw,
                                          int tag,
                                          int driving,
                                          double scale);

PGCS_API int pgcs_add_tangent_line_arc(pgcs_context* ctx,
                                       int line,
                                       int arc,
                                       int ccw,
                                       int tag,
                                       int driving,
                                       double scale);

PGCS_API int pgcs_add_tangent_circle_circle(pgcs_context* ctx,
                                            int circle1,
                                            int circle2,
                                            int tag,
                                            int driving,
                                            double scale);

PGCS_API int pgcs_add_tangent_circle_arc(pgcs_context* ctx,
                                         int circle,
                                         int arc,
                                         int tag,
                                         int driving,
                                         double scale);

PGCS_API int pgcs_add_tangent_arc_arc(pgcs_context* ctx,
                                      int arc1,
                                      int arc2,
                                      int tag,
                                      int driving,
                                      double scale);

/* ------------------------------------------------------------------------- */
/* Solving and diagnostics                                                  */
/* ------------------------------------------------------------------------- */

/* Returns PGCS_RESULT_SUCCESS, PGCS_RESULT_CONVERGED, PGCS_RESULT_FAILED
 * or a negative error code. The parameters are not updated until
 * pgcs_apply_solution() is called. */
PGCS_API int pgcs_solve(pgcs_context* ctx, int algorithm);

PGCS_API int pgcs_apply_solution(pgcs_context* ctx);

/* Recomputes DoF, conflicting and redundant tags on the current parameter
 * values. Call it after pgcs_apply_solution() to diagnose the solved geometry.
 * Returns the remaining degrees of freedom or a negative error code. */
PGCS_API int pgcs_diagnose(pgcs_context* ctx, int algorithm);

/* Recomputes DoF from the full (unreduced) Jacobian rank on the current
 * parameter values and refreshes redundant tags. Call after applying the
 * solution. Returns the remaining degrees of freedom or a negative error. */
PGCS_API int pgcs_diagnose_full(pgcs_context* ctx);

/* Remaining degrees of freedom, or -1 before diagnosis, or a negative error. */
PGCS_API int pgcs_dof(const pgcs_context* ctx);

PGCS_API int pgcs_has_conflicting(const pgcs_context* ctx);
PGCS_API int pgcs_has_redundant(const pgcs_context* ctx);
PGCS_API int pgcs_has_partially_redundant(const pgcs_context* ctx);

/* Copies up to `capacity` tags and returns the total number of tags. */
PGCS_API int pgcs_get_conflicting(const pgcs_context* ctx, int* tags, int capacity);
PGCS_API int pgcs_get_redundant(const pgcs_context* ctx, int* tags, int capacity);
PGCS_API int pgcs_get_partially_redundant(const pgcs_context* ctx, int* tags, int capacity);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // PLANEGCS_NATIVE_H
