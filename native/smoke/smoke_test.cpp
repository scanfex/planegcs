#include "planegcs_native.h"

#include <cmath>
#include <cstdio>

namespace
{

bool near(double value, double expected, double tolerance = 1e-6)
{
    return std::fabs(value - expected) <= tolerance;
}

int solve_coincidence()
{
    pgcs_context* context = pgcs_create();

    if (context == nullptr) {
        std::printf("coincidence: failed to create context\n");
        return 1;
    }

    const int x1 = pgcs_add_param(context, 0.0, 1);
    const int y1 = pgcs_add_param(context, 0.0, 1);
    const int x2 = pgcs_add_param(context, 10.0, 0);
    const int y2 = pgcs_add_param(context, 10.0, 0);

    const int p1 = pgcs_add_point(context, x1, y1);
    const int p2 = pgcs_add_point(context, x2, y2);

    const int tag = pgcs_add_coincident(context, p1, p2, 1, 1, 1.0);
    const int status = pgcs_solve(context, PGCS_ALGORITHM_DOG_LEG);
    const int applied = pgcs_apply_solution(context);

    double solved_x = 0.0;
    double solved_y = 0.0;

    pgcs_get_param(context, x2, &solved_x);
    pgcs_get_param(context, y2, &solved_y);

    const int dof = pgcs_diagnose(context, PGCS_ALGORITHM_DOG_LEG);

    std::printf("coincidence: tag=%d status=%d apply=%d dof=%d p2=(%g, %g) error='%s'\n",
                tag,
                status,
                applied,
                dof,
                solved_x,
                solved_y,
                pgcs_last_error(context));

    const bool ok = tag == 1 && status == PGCS_RESULT_SUCCESS && applied == 0 && dof == 0
                    && near(solved_x, 0.0) && near(solved_y, 0.0);

    pgcs_destroy(context);
    return ok ? 0 : 1;
}

int solve_distance()
{
    pgcs_context* context = pgcs_create();

    if (context == nullptr) {
        std::printf("distance: failed to create context\n");
        return 1;
    }

    const int x1 = pgcs_add_param(context, 0.0, 1);
    const int y1 = pgcs_add_param(context, 0.0, 1);
    const int x2 = pgcs_add_param(context, 10.0, 0);
    const int y2 = pgcs_add_param(context, 10.0, 0);
    const int distance = pgcs_add_param(context, 5.0, 1);

    const int p1 = pgcs_add_point(context, x1, y1);
    const int p2 = pgcs_add_point(context, x2, y2);

    const int tag = pgcs_add_p2p_distance(context, p1, p2, distance, 1, 1, 1.0);
    const int status = pgcs_solve(context, PGCS_ALGORITHM_DOG_LEG);
    const int applied = pgcs_apply_solution(context);

    double solved_x = 0.0;
    double solved_y = 0.0;

    pgcs_get_param(context, x2, &solved_x);
    pgcs_get_param(context, y2, &solved_y);

    const double actual = std::sqrt(solved_x * solved_x + solved_y * solved_y);
    const int dof = pgcs_diagnose(context, PGCS_ALGORITHM_DOG_LEG);

    std::printf("distance:    tag=%d status=%d apply=%d dof=%d p2=(%g, %g) |p2|=%g error='%s'\n",
                tag,
                status,
                applied,
                dof,
                solved_x,
                solved_y,
                actual,
                pgcs_last_error(context));

    const bool ok = tag == 1 && status == PGCS_RESULT_SUCCESS && applied == 0 && dof == 1
                    && near(actual, 5.0, 1e-4);

    pgcs_destroy(context);
    return ok ? 0 : 1;
}

int solve_p2l_distance_ccw_side(bool ccw, double expectedY)
{
    pgcs_context* context = pgcs_create();

    if (context == nullptr) {
        std::printf("p2l-ccw:    failed to create context\n");
        return 1;
    }

    const int x1 = pgcs_add_param(context, 0.0, 1);
    const int y1 = pgcs_add_param(context, 0.0, 1);
    const int x2 = pgcs_add_param(context, 10.0, 1);
    const int y2 = pgcs_add_param(context, 0.0, 1);
    const int pointX = pgcs_add_param(context, 0.0, 1);
    const int pointY = pgcs_add_param(context, 5.0, 0);
    const int distance = pgcs_add_param(context, 2.0, 1);

    const int p1 = pgcs_add_point(context, x1, y1);
    const int p2 = pgcs_add_point(context, x2, y2);
    const int point = pgcs_add_point(context, pointX, pointY);
    const int line = pgcs_add_line(context, p1, p2);

    const int tag = pgcs_add_p2l_distance(context, point, line, distance, ccw ? 1 : 0, 1, 1, 1.0);
    const int status = pgcs_solve(context, PGCS_ALGORITHM_DOG_LEG);
    const int applied = pgcs_apply_solution(context);

    double solvedY = 0.0;
    pgcs_get_param(context, pointY, &solvedY);

    std::printf("p2l-ccw(%d): tag=%d status=%d apply=%d y=%g error='%s'\n",
                ccw ? 1 : 0,
                tag,
                status,
                applied,
                solvedY,
                pgcs_last_error(context));

    const bool ok = tag == 1 && status == PGCS_RESULT_SUCCESS && applied == 0
                    && near(solvedY, expectedY, 1e-4);

    pgcs_destroy(context);
    return ok ? 0 : 1;
}

int solve_scale_free_shape()
{
    pgcs_context* context = pgcs_create();

    if (context == nullptr) {
        std::printf("scale-free: failed to create context\n");
        return 1;
    }

    const int x1 = pgcs_add_param(context, 0.0, 1);
    const int y1 = pgcs_add_param(context, 0.0, 1);
    const int x2 = pgcs_add_param(context, 2.0, 0);
    const int y2 = pgcs_add_param(context, 0.0, 1);
    const int x3 = pgcs_add_param(context, 2.69, 0);
    const int y3 = pgcs_add_param(context, 0.76, 0);
    const int x4 = pgcs_add_param(context, 0.68, 0);
    const int y4 = pgcs_add_param(context, 0.68, 0);
    const int angle = pgcs_add_param(context, 135.0 * 3.14159265358979323846 / 180.0, 1);

    const int p1 = pgcs_add_point(context, x1, y1);
    const int p2 = pgcs_add_point(context, x2, y2);
    const int p3 = pgcs_add_point(context, x3, y3);
    const int p4 = pgcs_add_point(context, x4, y4);

    const int l1 = pgcs_add_line(context, p1, p2);
    const int l2 = pgcs_add_line(context, p2, p3);
    const int l3 = pgcs_add_line(context, p3, p4);
    const int l4 = pgcs_add_line(context, p4, p1);

    const int tags = pgcs_add_parallel(context, l1, l3, 1, 1, 1.0)
                     + pgcs_add_parallel(context, l2, l4, 2, 1, 1.0)
                     + pgcs_add_equal_length(context, l1, l3, 3, 1, 1.0)
                     + pgcs_add_equal_length(context, l2, l4, 4, 1, 1.0)
                     + pgcs_add_l2l_angle(context, l4, l1, angle, 5, 1, 1.0);

    const int status = pgcs_solve(context, PGCS_ALGORITHM_DOG_LEG);
    const int applied = pgcs_apply_solution(context);
    const int dof_before_diagnose = pgcs_dof(context);
    const int dof = pgcs_diagnose(context, PGCS_ALGORITHM_DOG_LEG);
    const int redundant = pgcs_has_redundant(context);
    const int dof_full = pgcs_diagnose_full(context);

    std::printf("scale-free:  tags=%d status=%d apply=%d dof_before=%d dof=%d dof_full=%d redundant=%d error='%s'\n",
                tags,
                status,
                applied,
                dof_before_diagnose,
                dof,
                dof_full,
                redundant,
                pgcs_last_error(context));

    const bool ok = tags > 0 && status == PGCS_RESULT_SUCCESS && applied == 0
                    && dof_before_diagnose == 0 && dof == 2 && dof_full == 2;

    pgcs_destroy(context);
    return ok ? 0 : 1;
}

int cancellation_status()
{
    pgcs_context* context = pgcs_create();
    if (context == nullptr) {
        return 1;
    }

    const int x = pgcs_add_param(context, 0.0, 0);
    const int y = pgcs_add_param(context, 0.0, 0);
    const int fixedX = pgcs_add_param(context, 3.0, 1);
    const int fixedY = pgcs_add_param(context, 4.0, 1);
    const int p1 = pgcs_add_point(context, x, y);
    const int p2 = pgcs_add_point(context, fixedX, fixedY);
    const int distance = pgcs_add_param(context, 5.0, 1);
    const int tag = pgcs_add_p2p_distance(context, p1, p2, distance, 1, 1, 1.0);

    pgcs_request_cancel(context);
    const int requested = pgcs_is_cancel_requested(context);
    const int cancelledSolve = pgcs_solve(context, PGCS_ALGORITHM_DOG_LEG);
    const int cancelledDiagnose = pgcs_diagnose(context, PGCS_ALGORITHM_DOG_LEG);
    const int cancelledDiagnoseFull = pgcs_diagnose_full(context);

    pgcs_clear_cancel(context);
    const int cleared = pgcs_is_cancel_requested(context);
    const int status = pgcs_solve(context, PGCS_ALGORITHM_DOG_LEG);
    const int applied = pgcs_apply_solution(context);
    const int dof = pgcs_diagnose(context, PGCS_ALGORITHM_DOG_LEG);
    const int dofFull = pgcs_diagnose_full(context);

    std::printf(
        "cancel:      tag=%d requested=%d solve=%d diagnose=%d diagnose_full=%d "
        "cleared=%d status=%d apply=%d dof=%d dof_full=%d error='%s'\n",
        tag,
        requested,
        cancelledSolve,
        cancelledDiagnose,
        cancelledDiagnoseFull,
        cleared,
        status,
        applied,
        dof,
        dofFull,
        pgcs_last_error(context)
    );

    const bool ok = tag > 0 && requested == 1 && cancelledSolve == PGCS_RESULT_CANCELLED
                    && cancelledDiagnose == PGCS_ERROR_CANCELLED
                    && cancelledDiagnoseFull == PGCS_ERROR_CANCELLED && cleared == 0
                    && status == PGCS_RESULT_SUCCESS && applied == 0 && dof == 1 && dofFull == 1;

    pgcs_destroy(context);
    return ok ? 0 : 1;
}

}  // namespace

int main()
{
    std::printf("planegcs_native version: %s\n", pgcs_version());

    const int failures = solve_coincidence() + solve_distance()
                         + solve_p2l_distance_ccw_side(true, 2.0)
                         + solve_p2l_distance_ccw_side(false, -2.0)
                         + solve_scale_free_shape()
                         + cancellation_status();

    if (failures == 0) {
        std::printf("smoke test: OK\n");
    }
    else {
        std::printf("smoke test: %d failure(s)\n", failures);
    }

    return failures == 0 ? 0 : 1;
}
