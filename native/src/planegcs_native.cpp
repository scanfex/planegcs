#include "planegcs_native.h"

#include "GCS.h"

#include <cstdint>
#include <new>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace
{

using namespace GCS;

class pgcs_error : public std::runtime_error
{
public:
    pgcs_error(int code, const std::string& message)
        : std::runtime_error(message)
        , code(code)
    {
    }

    int code;
};

[[noreturn]] void fail(int code, const char* message)
{
    throw pgcs_error(code, message);
}

struct Geometry
{
    enum class Kind
    {
        Point,
        Line,
        Circle,
        Arc
    };

    explicit Geometry(const Point& value)
        : kind(Kind::Point)
        , point(value)
    {
    }

    explicit Geometry(const Line& value)
        : kind(Kind::Line)
        , line(value)
    {
    }

    explicit Geometry(const Circle& value)
        : kind(Kind::Circle)
        , circle(value)
    {
    }

    explicit Geometry(const Arc& value)
        : kind(Kind::Arc)
        , arc(value)
    {
    }

    Kind kind;
    Point point;
    Line line;
    Circle circle;
    Arc arc;
};

}  // namespace

struct pgcs_context
{
    System system;
    std::vector<double*> params;
    std::vector<std::uint8_t> is_fixed;
    std::vector<Geometry> geometry;
    std::string last_error;
};

namespace
{

template<typename Function>
int guarded(pgcs_context* context, Function&& function)
{
    if (context == nullptr) {
        return PGCS_ERROR_INVALID_ARGUMENT;
    }

    try {
        return function();
    }
    catch (const pgcs_error& error) {
        context->last_error = error.what();
        return error.code;
    }
    catch (const std::exception& error) {
        context->last_error = error.what();
        return PGCS_ERROR_UNKNOWN;
    }
    catch (...) {
        context->last_error = "unknown native error";
        return PGCS_ERROR_UNKNOWN;
    }
}

double* require_param(pgcs_context* context, int index)
{
    if (index < 0 || index >= static_cast<int>(context->params.size())) {
        fail(PGCS_ERROR_INVALID_ARGUMENT, "parameter index is out of range");
    }

    return context->params[static_cast<std::size_t>(index)];
}

const double* require_param(const pgcs_context* context, int index)
{
    if (index < 0 || index >= static_cast<int>(context->params.size())) {
        fail(PGCS_ERROR_INVALID_ARGUMENT, "parameter index is out of range");
    }

    return context->params[static_cast<std::size_t>(index)];
}

Geometry& require_geometry(pgcs_context* context, int handle)
{
    if (handle < 0 || handle >= static_cast<int>(context->geometry.size())) {
        fail(PGCS_ERROR_INVALID_HANDLE, "geometry handle is out of range");
    }

    return context->geometry[static_cast<std::size_t>(handle)];
}

const Geometry& require_geometry(const pgcs_context* context, int handle)
{
    if (handle < 0 || handle >= static_cast<int>(context->geometry.size())) {
        fail(PGCS_ERROR_INVALID_HANDLE, "geometry handle is out of range");
    }

    return context->geometry[static_cast<std::size_t>(handle)];
}

Point& require_point(pgcs_context* context, int handle)
{
    Geometry& geometry = require_geometry(context, handle);

    if (geometry.kind != Geometry::Kind::Point) {
        fail(PGCS_ERROR_WRONG_GEOMETRY_TYPE, "geometry handle is not a point");
    }

    return geometry.point;
}

Line& require_line(pgcs_context* context, int handle)
{
    Geometry& geometry = require_geometry(context, handle);

    if (geometry.kind != Geometry::Kind::Line) {
        fail(PGCS_ERROR_WRONG_GEOMETRY_TYPE, "geometry handle is not a line");
    }

    return geometry.line;
}

Circle& require_circle(pgcs_context* context, int handle)
{
    Geometry& geometry = require_geometry(context, handle);

    if (geometry.kind == Geometry::Kind::Circle) {
        return geometry.circle;
    }

    if (geometry.kind == Geometry::Kind::Arc) {
        return geometry.arc;
    }

    fail(PGCS_ERROR_WRONG_GEOMETRY_TYPE, "geometry handle is neither a circle nor an arc");
}

Arc& require_arc(pgcs_context* context, int handle)
{
    Geometry& geometry = require_geometry(context, handle);

    if (geometry.kind != Geometry::Kind::Arc) {
        fail(PGCS_ERROR_WRONG_GEOMETRY_TYPE, "geometry handle is not an arc");
    }

    return geometry.arc;
}

template<typename AddConstraint>
int add_constraint(pgcs_context* context, int tag, int driving, double scale, AddConstraint&& add)
{
    return guarded(context, [&]() -> int {
        if (tag < 0) {
            fail(PGCS_ERROR_INVALID_ARGUMENT, "constraint tag must be non-negative");
        }

        if (scale <= 0.0) {
            fail(PGCS_ERROR_INVALID_ARGUMENT, "constraint scale must be positive");
        }

        const int index = add();

        if (index < 0) {
            fail(PGCS_ERROR_GCS, "planegcs rejected the constraint");
        }

        if (scale != 1.0) {
            context->system.rescaleConstraint(index, scale);
        }

        return tag;
    });
}

int required_geometry_param_count(Geometry::Kind kind)
{
    switch (kind) {
        case Geometry::Kind::Point:
            return 2;
        case Geometry::Kind::Line:
            return 4;
        case Geometry::Kind::Circle:
            return 3;
        case Geometry::Kind::Arc:
            return 9;
    }

    return 0;
}

std::vector<int> read_tags(const pgcs_context* context, void (System::*getter)(VEC_I&) const)
{
    VEC_I tags;
    (const_cast<System&>(context->system).*getter)(tags);
    return std::vector<int>(tags.begin(), tags.end());
}

int copy_tags(const std::vector<int>& tags, int* output, int capacity)
{
    if (capacity < 0) {
        fail(PGCS_ERROR_INVALID_ARGUMENT, "tag buffer capacity must be non-negative");
    }

    if (output != nullptr) {
        const std::size_t count =
            tags.size() < static_cast<std::size_t>(capacity)
                ? tags.size()
                : static_cast<std::size_t>(capacity);

        for (std::size_t index = 0; index < count; ++index) {
            output[index] = tags[index];
        }
    }

    return static_cast<int>(tags.size());
}

}  // namespace

extern "C"
{

PGCS_API const char* pgcs_version(void)
{
    return "0.1.1";
}

PGCS_API pgcs_context* pgcs_create(void)
{
    try {
        return new pgcs_context();
    }
    catch (...) {
        return nullptr;
    }
}

PGCS_API void pgcs_destroy(pgcs_context* context)
{
    if (context == nullptr) {
        return;
    }

    pgcs_clear(context);
    delete context;
}

PGCS_API void pgcs_clear(pgcs_context* context)
{
    if (context == nullptr) {
        return;
    }

    try {
        context->system.clear();

        for (double* parameter : context->params) {
            delete parameter;
        }

        context->params.clear();
        context->is_fixed.clear();
        context->geometry.clear();
        context->last_error.clear();
    }
    catch (const std::exception& error) {
        context->last_error = error.what();
    }
    catch (...) {
        context->last_error = "unknown native error";
    }
}

PGCS_API const char* pgcs_last_error(const pgcs_context* context)
{
    if (context == nullptr) {
        return "";
    }

    return context->last_error.c_str();
}

PGCS_API int pgcs_add_param(pgcs_context* context, double value, int is_fixed)
{
    return guarded(context, [&]() -> int {
        context->params.reserve(context->params.size() + 1);
        context->is_fixed.reserve(context->is_fixed.size() + 1);

        double* parameter = new double(value);
        context->params.push_back(parameter);
        context->is_fixed.push_back(is_fixed != 0 ? 1 : 0);

        return static_cast<int>(context->params.size()) - 1;
    });
}

PGCS_API int pgcs_param_count(const pgcs_context* context)
{
    if (context == nullptr) {
        return PGCS_ERROR_INVALID_ARGUMENT;
    }

    return static_cast<int>(context->params.size());
}

PGCS_API int pgcs_get_param(const pgcs_context* context, int index, double* value)
{
    return guarded(const_cast<pgcs_context*>(context), [&]() -> int {
        if (value == nullptr) {
            fail(PGCS_ERROR_INVALID_ARGUMENT, "value must not be null");
        }

        *value = *require_param(context, index);
        return 0;
    });
}

PGCS_API int pgcs_set_param(pgcs_context* context, int index, double value, int is_fixed)
{
    return guarded(context, [&]() -> int {
        *require_param(context, index) = value;
        context->is_fixed[static_cast<std::size_t>(index)] = is_fixed != 0 ? 1 : 0;
        return 0;
    });
}

PGCS_API int pgcs_is_param_fixed(const pgcs_context* context, int index)
{
    return guarded(const_cast<pgcs_context*>(context), [&]() -> int {
        require_param(context, index);
        return context->is_fixed[static_cast<std::size_t>(index)] != 0 ? 1 : 0;
    });
}

PGCS_API int pgcs_set_max_iterations(pgcs_context* context, int max_iterations)
{
    return guarded(context, [&]() -> int {
        if (max_iterations <= 0) {
            fail(PGCS_ERROR_INVALID_ARGUMENT, "max_iterations must be positive");
        }

        context->system.maxIter = max_iterations;
        return 0;
    });
}

PGCS_API int pgcs_get_max_iterations(const pgcs_context* context, int* max_iterations)
{
    return guarded(const_cast<pgcs_context*>(context), [&]() -> int {
        if (max_iterations == nullptr) {
            fail(PGCS_ERROR_INVALID_ARGUMENT, "max_iterations must not be null");
        }

        *max_iterations = context->system.maxIter;
        return 0;
    });
}

PGCS_API int pgcs_set_convergence_threshold(pgcs_context* context, double threshold)
{
    return guarded(context, [&]() -> int {
        if (!(threshold > 0.0)) {
            fail(PGCS_ERROR_INVALID_ARGUMENT, "threshold must be positive");
        }

        context->system.convergence = threshold;
        return 0;
    });
}

PGCS_API int pgcs_get_convergence_threshold(const pgcs_context* context, double* threshold)
{
    return guarded(const_cast<pgcs_context*>(context), [&]() -> int {
        if (threshold == nullptr) {
            fail(PGCS_ERROR_INVALID_ARGUMENT, "threshold must not be null");
        }

        *threshold = context->system.convergence;
        return 0;
    });
}

PGCS_API int pgcs_set_debug_mode(pgcs_context* context, int debug_mode)
{
    return guarded(context, [&]() -> int {
        if (debug_mode < 0 || debug_mode > 2) {
            fail(PGCS_ERROR_INVALID_ARGUMENT, "debug_mode must be 0, 1 or 2");
        }

        context->system.debugMode = static_cast<DebugMode>(debug_mode);
        return 0;
    });
}

PGCS_API int pgcs_get_debug_mode(const pgcs_context* context, int* debug_mode)
{
    return guarded(const_cast<pgcs_context*>(context), [&]() -> int {
        if (debug_mode == nullptr) {
            fail(PGCS_ERROR_INVALID_ARGUMENT, "debug_mode must not be null");
        }

        *debug_mode = static_cast<int>(context->system.debugMode);
        return 0;
    });
}

PGCS_API int pgcs_add_point(pgcs_context* context, int x_param, int y_param)
{
    return guarded(context, [&]() -> int {
        const Point point(require_param(context, x_param), require_param(context, y_param));
        context->geometry.emplace_back(point);
        return static_cast<int>(context->geometry.size()) - 1;
    });
}

PGCS_API int pgcs_add_line(pgcs_context* context, int p1, int p2)
{
    return guarded(context, [&]() -> int {
        Line line;
        line.p1 = require_point(context, p1);
        line.p2 = require_point(context, p2);
        context->geometry.emplace_back(line);
        return static_cast<int>(context->geometry.size()) - 1;
    });
}

PGCS_API int pgcs_add_circle(pgcs_context* context, int center, int radius_param)
{
    return guarded(context, [&]() -> int {
        Circle circle;
        circle.center = require_point(context, center);
        circle.rad = require_param(context, radius_param);
        context->geometry.emplace_back(circle);
        return static_cast<int>(context->geometry.size()) - 1;
    });
}

PGCS_API int pgcs_add_arc(pgcs_context* context,
                          int center,
                          int start,
                          int end,
                          int start_angle_param,
                          int end_angle_param,
                          int radius_param)
{
    return guarded(context, [&]() -> int {
        Arc arc;
        arc.center = require_point(context, center);
        arc.start = require_point(context, start);
        arc.end = require_point(context, end);
        arc.startAngle = require_param(context, start_angle_param);
        arc.endAngle = require_param(context, end_angle_param);
        arc.rad = require_param(context, radius_param);
        context->geometry.emplace_back(arc);
        return static_cast<int>(context->geometry.size()) - 1;
    });
}

PGCS_API int pgcs_get_geometry_params(const pgcs_context* context,
                                      int handle,
                                      double* values,
                                      int capacity)
{
    return guarded(const_cast<pgcs_context*>(context), [&]() -> int {
        const Geometry& geometry = require_geometry(context, handle);
        const int required = required_geometry_param_count(geometry.kind);

        if (values == nullptr) {
            return required;
        }

        if (capacity < required) {
            fail(PGCS_ERROR_INVALID_ARGUMENT, "values buffer is too small");
        }

        switch (geometry.kind) {
            case Geometry::Kind::Point:
                values[0] = *geometry.point.x;
                values[1] = *geometry.point.y;
                break;
            case Geometry::Kind::Line:
                values[0] = *geometry.line.p1.x;
                values[1] = *geometry.line.p1.y;
                values[2] = *geometry.line.p2.x;
                values[3] = *geometry.line.p2.y;
                break;
            case Geometry::Kind::Circle:
                values[0] = *geometry.circle.center.x;
                values[1] = *geometry.circle.center.y;
                values[2] = *geometry.circle.rad;
                break;
            case Geometry::Kind::Arc:
                values[0] = *geometry.arc.center.x;
                values[1] = *geometry.arc.center.y;
                values[2] = *geometry.arc.start.x;
                values[3] = *geometry.arc.start.y;
                values[4] = *geometry.arc.end.x;
                values[5] = *geometry.arc.end.y;
                values[6] = *geometry.arc.startAngle;
                values[7] = *geometry.arc.endAngle;
                values[8] = *geometry.arc.rad;
                break;
        }

        return required;
    });
}

PGCS_API int pgcs_add_coincident(pgcs_context* context,
                                 int p1,
                                 int p2,
                                 int tag,
                                 int driving,
                                 double scale)
{
    return add_constraint(context, tag, driving, scale, [&]() {
        return context->system.addConstraintP2PCoincident(
            require_point(context, p1), require_point(context, p2), tag, driving != 0);
    });
}

PGCS_API int pgcs_add_p2p_distance(pgcs_context* context,
                                   int p1,
                                   int p2,
                                   int distance_param,
                                   int tag,
                                   int driving,
                                   double scale)
{
    return add_constraint(context, tag, driving, scale, [&]() {
        return context->system.addConstraintP2PDistance(require_point(context, p1),
                                                        require_point(context, p2),
                                                        require_param(context, distance_param),
                                                        tag,
                                                        driving != 0);
    });
}

PGCS_API int pgcs_add_p2l_distance(pgcs_context* context,
                                   int point,
                                   int line,
                                   int distance_param,
                                   int tag,
                                   int driving,
                                   double scale)
{
    return add_constraint(context, tag, driving, scale, [&]() {
        return context->system.addConstraintP2LDistance(require_point(context, point),
                                                        require_line(context, line),
                                                        require_param(context, distance_param),
                                                        tag,
                                                        driving != 0);
    });
}

PGCS_API int pgcs_add_p2c_distance(pgcs_context* context,
                                   int point,
                                   int circle,
                                   int distance_param,
                                   int tag,
                                   int driving,
                                   double scale)
{
    return add_constraint(context, tag, driving, scale, [&]() {
        return context->system.addConstraintP2CDistance(require_point(context, point),
                                                        require_circle(context, circle),
                                                        require_param(context, distance_param),
                                                        tag,
                                                        driving != 0);
    });
}

PGCS_API int pgcs_add_c2c_distance(pgcs_context* context,
                                   int circle1,
                                   int circle2,
                                   int distance_param,
                                   int tag,
                                   int driving,
                                   double scale)
{
    return add_constraint(context, tag, driving, scale, [&]() {
        return context->system.addConstraintC2CDistance(require_circle(context, circle1),
                                                        require_circle(context, circle2),
                                                        require_param(context, distance_param),
                                                        tag,
                                                        driving != 0);
    });
}

PGCS_API int pgcs_add_c2l_distance(pgcs_context* context,
                                   int circle,
                                   int line,
                                   int distance_param,
                                   int tag,
                                   int driving,
                                   double scale)
{
    return add_constraint(context, tag, driving, scale, [&]() {
        return context->system.addConstraintC2LDistance(require_circle(context, circle),
                                                        require_line(context, line),
                                                        require_param(context, distance_param),
                                                        tag,
                                                        driving != 0);
    });
}

PGCS_API int pgcs_add_l2l_angle(pgcs_context* context,
                                int line1,
                                int line2,
                                int angle_param,
                                int tag,
                                int driving,
                                double scale)
{
    return add_constraint(context, tag, driving, scale, [&]() {
        return context->system.addConstraintL2LAngle(require_line(context, line1),
                                                     require_line(context, line2),
                                                     require_param(context, angle_param),
                                                     tag,
                                                     driving != 0);
    });
}

PGCS_API int pgcs_add_parallel(pgcs_context* context,
                               int line1,
                               int line2,
                               int tag,
                               int driving,
                               double scale)
{
    return add_constraint(context, tag, driving, scale, [&]() {
        return context->system.addConstraintParallel(
            require_line(context, line1), require_line(context, line2), tag, driving != 0);
    });
}

PGCS_API int pgcs_add_perpendicular(pgcs_context* context,
                                    int line1,
                                    int line2,
                                    int tag,
                                    int driving,
                                    double scale)
{
    return add_constraint(context, tag, driving, scale, [&]() {
        return context->system.addConstraintPerpendicular(
            require_line(context, line1), require_line(context, line2), tag, driving != 0);
    });
}

PGCS_API int pgcs_add_point_on_line(pgcs_context* context,
                                    int point,
                                    int line,
                                    int tag,
                                    int driving,
                                    double scale)
{
    return add_constraint(context, tag, driving, scale, [&]() {
        return context->system.addConstraintPointOnLine(
            require_point(context, point), require_line(context, line), tag, driving != 0);
    });
}

PGCS_API int pgcs_add_point_on_circle(pgcs_context* context,
                                      int point,
                                      int circle,
                                      int tag,
                                      int driving,
                                      double scale)
{
    return add_constraint(context, tag, driving, scale, [&]() {
        return context->system.addConstraintPointOnCircle(
            require_point(context, point), require_circle(context, circle), tag, driving != 0);
    });
}

PGCS_API int pgcs_add_point_on_arc(pgcs_context* context,
                                   int point,
                                   int arc,
                                   int tag,
                                   int driving,
                                   double scale)
{
    return add_constraint(context, tag, driving, scale, [&]() {
        return context->system.addConstraintPointOnArc(
            require_point(context, point), require_arc(context, arc), tag, driving != 0);
    });
}

PGCS_API int pgcs_add_arc_rules(pgcs_context* context,
                                int arc,
                                int tag,
                                int driving,
                                double scale)
{
    return add_constraint(context, tag, driving, scale, [&]() {
        return context->system.addConstraintArcRules(require_arc(context, arc), tag, driving != 0);
    });
}

PGCS_API int pgcs_add_equal(pgcs_context* context,
                            int param1,
                            int param2,
                            int tag,
                            int driving,
                            double scale)
{
    return add_constraint(context, tag, driving, scale, [&]() {
        return context->system.addConstraintEqual(
            require_param(context, param1), require_param(context, param2), tag, driving != 0);
    });
}

PGCS_API int pgcs_add_equal_length(pgcs_context* context,
                                   int line1,
                                   int line2,
                                   int tag,
                                   int driving,
                                   double scale)
{
    return add_constraint(context, tag, driving, scale, [&]() {
        return context->system.addConstraintEqualLength(
            require_line(context, line1), require_line(context, line2), tag, driving != 0);
    });
}

PGCS_API int pgcs_add_equal_radius(pgcs_context* context,
                                   int circle1,
                                   int circle2,
                                   int tag,
                                   int driving,
                                   double scale)
{
    return add_constraint(context, tag, driving, scale, [&]() {
        return context->system.addConstraintEqualRadius(
            require_circle(context, circle1), require_circle(context, circle2), tag, driving != 0);
    });
}

PGCS_API int pgcs_add_circle_radius(pgcs_context* context,
                                    int circle,
                                    int radius_param,
                                    int tag,
                                    int driving,
                                    double scale)
{
    return add_constraint(context, tag, driving, scale, [&]() {
        return context->system.addConstraintCircleRadius(require_circle(context, circle),
                                                         require_param(context, radius_param),
                                                         tag,
                                                         driving != 0);
    });
}

PGCS_API int pgcs_add_arc_radius(pgcs_context* context,
                                 int arc,
                                 int radius_param,
                                 int tag,
                                 int driving,
                                 double scale)
{
    return add_constraint(context, tag, driving, scale, [&]() {
        return context->system.addConstraintArcRadius(
            require_arc(context, arc), require_param(context, radius_param), tag, driving != 0);
    });
}

PGCS_API int pgcs_add_circle_diameter(pgcs_context* context,
                                      int circle,
                                      int diameter_param,
                                      int tag,
                                      int driving,
                                      double scale)
{
    return add_constraint(context, tag, driving, scale, [&]() {
        return context->system.addConstraintCircleDiameter(require_circle(context, circle),
                                                           require_param(context, diameter_param),
                                                           tag,
                                                           driving != 0);
    });
}

PGCS_API int pgcs_add_arc_diameter(pgcs_context* context,
                                   int arc,
                                   int diameter_param,
                                   int tag,
                                   int driving,
                                   double scale)
{
    return add_constraint(context, tag, driving, scale, [&]() {
        return context->system.addConstraintArcDiameter(
            require_arc(context, arc), require_param(context, diameter_param), tag, driving != 0);
    });
}

PGCS_API int pgcs_add_difference(pgcs_context* context,
                                 int param1,
                                 int param2,
                                 int difference_param,
                                 int tag,
                                 int driving,
                                 double scale)
{
    return add_constraint(context, tag, driving, scale, [&]() {
        return context->system.addConstraintDifference(require_param(context, param1),
                                                       require_param(context, param2),
                                                       require_param(context, difference_param),
                                                       tag,
                                                       driving != 0);
    });
}

PGCS_API int pgcs_add_symmetric_line(pgcs_context* context,
                                     int p1,
                                     int p2,
                                     int line,
                                     int tag,
                                     int driving,
                                     double scale)
{
    return add_constraint(context, tag, driving, scale, [&]() {
        return context->system.addConstraintP2PSymmetric(require_point(context, p1),
                                                         require_point(context, p2),
                                                         require_line(context, line),
                                                         tag,
                                                         driving != 0);
    });
}

PGCS_API int pgcs_add_symmetric_point(pgcs_context* context,
                                      int p1,
                                      int p2,
                                      int point,
                                      int tag,
                                      int driving,
                                      double scale)
{
    return add_constraint(context, tag, driving, scale, [&]() {
        return context->system.addConstraintP2PSymmetric(require_point(context, p1),
                                                         require_point(context, p2),
                                                         require_point(context, point),
                                                         tag,
                                                         driving != 0);
    });
}

PGCS_API int pgcs_add_tangent_line_circle(pgcs_context* context,
                                          int line,
                                          int circle,
                                          int tag,
                                          int driving,
                                          double scale)
{
    return add_constraint(context, tag, driving, scale, [&]() {
        return context->system.addConstraintTangent(
            require_line(context, line), require_circle(context, circle), tag, driving != 0);
    });
}

PGCS_API int pgcs_add_tangent_line_arc(pgcs_context* context,
                                       int line,
                                       int arc,
                                       int tag,
                                       int driving,
                                       double scale)
{
    return add_constraint(context, tag, driving, scale, [&]() {
        return context->system.addConstraintTangent(
            require_line(context, line), require_arc(context, arc), tag, driving != 0);
    });
}

PGCS_API int pgcs_add_tangent_circle_circle(pgcs_context* context,
                                            int circle1,
                                            int circle2,
                                            int tag,
                                            int driving,
                                            double scale)
{
    return add_constraint(context, tag, driving, scale, [&]() {
        return context->system.addConstraintTangent(
            require_circle(context, circle1), require_circle(context, circle2), tag, driving != 0);
    });
}

PGCS_API int pgcs_add_tangent_circle_arc(pgcs_context* context,
                                         int circle,
                                         int arc,
                                         int tag,
                                         int driving,
                                         double scale)
{
    return add_constraint(context, tag, driving, scale, [&]() {
        return context->system.addConstraintTangent(
            require_circle(context, circle), require_arc(context, arc), tag, driving != 0);
    });
}

PGCS_API int pgcs_add_tangent_arc_arc(pgcs_context* context,
                                      int arc1,
                                      int arc2,
                                      int tag,
                                      int driving,
                                      double scale)
{
    return add_constraint(context, tag, driving, scale, [&]() {
        return context->system.addConstraintTangent(
            require_arc(context, arc1), require_arc(context, arc2), tag, driving != 0);
    });
}

PGCS_API int pgcs_solve(pgcs_context* context, int algorithm)
{
    return guarded(context, [&]() -> int {
        if (algorithm < PGCS_ALGORITHM_BFGS || algorithm > PGCS_ALGORITHM_DOG_LEG) {
            fail(PGCS_ERROR_INVALID_ARGUMENT, "unknown algorithm");
        }

        std::vector<double*> unknowns;
        unknowns.reserve(context->params.size());

        for (std::size_t index = 0; index < context->params.size(); ++index) {
            if (context->is_fixed[index] == 0) {
                unknowns.push_back(context->params[index]);
            }
        }

        return context->system.solve(
            unknowns, true, static_cast<Algorithm>(algorithm));
    });
}

PGCS_API int pgcs_apply_solution(pgcs_context* context)
{
    return guarded(context, [&]() -> int {
        context->system.applySolution();
        return 0;
    });
}

PGCS_API int pgcs_diagnose(pgcs_context* context, int algorithm)
{
    return guarded(context, [&]() -> int {
        if (algorithm < PGCS_ALGORITHM_BFGS || algorithm > PGCS_ALGORITHM_DOG_LEG) {
            fail(PGCS_ERROR_INVALID_ARGUMENT, "unknown algorithm");
        }

        context->system.invalidatedDiagnosis();
        context->system.initSolution(static_cast<Algorithm>(algorithm));
        return context->system.dofsNumber();
    });
}

PGCS_API int pgcs_diagnose_full(pgcs_context* context)
{
    return guarded(context, [&]() -> int {
        return context->system.diagnoseFull();
    });
}

PGCS_API int pgcs_dof(const pgcs_context* context)
{
    if (context == nullptr) {
        return PGCS_ERROR_INVALID_ARGUMENT;
    }

    return context->system.dofsNumber();
}

PGCS_API int pgcs_has_conflicting(const pgcs_context* context)
{
    return guarded(const_cast<pgcs_context*>(context), [&]() -> int {
        return context->system.hasConflicting() ? 1 : 0;
    });
}

PGCS_API int pgcs_has_redundant(const pgcs_context* context)
{
    return guarded(const_cast<pgcs_context*>(context), [&]() -> int {
        return context->system.hasRedundant() ? 1 : 0;
    });
}

PGCS_API int pgcs_has_partially_redundant(const pgcs_context* context)
{
    return guarded(const_cast<pgcs_context*>(context), [&]() -> int {
        return context->system.hasPartiallyRedundant() ? 1 : 0;
    });
}

PGCS_API int pgcs_get_conflicting(const pgcs_context* context, int* tags, int capacity)
{
    return guarded(const_cast<pgcs_context*>(context), [&]() -> int {
        const std::vector<int> values = read_tags(context, &System::getConflicting);
        return copy_tags(values, tags, capacity);
    });
}

PGCS_API int pgcs_get_redundant(const pgcs_context* context, int* tags, int capacity)
{
    return guarded(const_cast<pgcs_context*>(context), [&]() -> int {
        const std::vector<int> values = read_tags(context, &System::getRedundant);
        return copy_tags(values, tags, capacity);
    });
}

PGCS_API int pgcs_get_partially_redundant(const pgcs_context* context, int* tags, int capacity)
{
    return guarded(const_cast<pgcs_context*>(context), [&]() -> int {
        const std::vector<int> values = read_tags(context, &System::getPartiallyRedundant);
        return copy_tags(values, tags, capacity);
    });
}

}  // extern "C"
