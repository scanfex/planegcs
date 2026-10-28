# Ограничения PlaneGCS

Документ описывает ограничения, которые есть в текущем vendored-снимке
PlaneGCS из FreeCAD `main`.

- Снимок FreeCAD: `74168a8ae2d0c1a6e3006297673a871e8c1c8399` (2026-10-28).
- Исходники: `planegcs/Constraints.h`, `planegcs/Constraints.cpp`,
  `planegcs/GCS.h`, `planegcs/GCS.cpp`.
- Пространство имён: `GCS`.
- Язык сборки: C++23 (используются `std::unreachable`, `std::ranges` и
  `std::format` через `Base::Console`).

Обозначения:

- `scale` — нормировочный коэффициент конкретного скалярного ограничения.
  Солвер минимизирует `e(x) = scale * f(x)`; `e = 0` означает выполнение
  ограничения.
- `tagId` — пользовательский тег ограничения. Несколько скалярных ограничений
  могут иметь один тег (так устроены составные ограничения).
- `driving = true` — ограничение управляет геометрией.
  `driving = false` — driven-ограничение: его числовой параметр не является
  неизвестным, а вычисляется методом `Constraint::evaluate()` после решения.
- `internalAlignment` — служебный флаг для internal alignment ограничений
  (оси/фокусы эллипсов и гипербол, контрольные точки B-сплайнов).
- `ccw` — сторона направленной прямой `p1 -> p2`: `true` — положительная
  (против часовой стрелки) сторона, `false` — противоположная.
- `internal` — для расстояний «окружность–окружность/прямая»: измерять зазор
  снаружи или внутри окружности.

### Что важно в текущем снимке

- `addConstraintP2LDistance(..., bool ccw, ...)` — расстояние «точка–прямая»
  стало знаковым; `ccw` задаёт сторону прямой.
- `addConstraintTangent(Line&, Circle&, bool ccw, ...)` и
  `addConstraintTangent(Line&, Arc&, bool ccw, ...)` — касание
  «прямая–окружность/дуга» тоже требует `ccw`.
- `addConstraintC2LDistance(..., bool ccw, bool internal, ...)` — знаковое
  расстояние «окружность–прямая» с выбором внешнего/внутреннего зазора.
- `addConstraintC2CDistance(..., std::optional<bool> c1bigger, ...)` — явный
  выбор большей окружности для внутреннего случая.
- В native C API экспортированы `ccw` и `internal`; параметр `c1bigger` пока
  не выведен — native-обёртка передаёт `std::nullopt`. См. раздел 12.

## 1. Как устроено ограничение в коде

Базовый класс — `GCS::Constraint` (`Constraints.h`). У него есть:

- `error()` — скалярная невязка;
- `grad(double* param)` — производная невязки по параметру;
- `errorgrad(err, grad, param)` — общая реализация для сложных ограничений;
- `rescale(coef)` — пересчёт `scale`;
- `maxStep(dir, lim)` — ограничение шага (например, для углов);
- `evaluate()` — вычисление значения для driven-ограничения (реализовано не
  для всех ограничений);
- `getTypeId()` — значение `ConstraintType`.

Солвер решает систему

```text
e_i(x) = 0,  i = 1..m
```

относительно вектора параметров `x`. Для переопределённых систем используется
минимизация суммы квадратов невязок. Метод решения задаётся `Algorithm`:
`BFGS`, `LevenbergMarquardt` или `DogLeg`.

### 1.1. Теги, индексы и диагностика

- `System::addConstraint(Constraint*)` возвращает **индекс** ограничения в
  `clist`, а не `tagId`.
- Фабричные методы `System::addConstraint...()` принимают `tagId` и возвращают
  индекс последнего созданного скалярного ограничения.
- `System::rescaleConstraint(int id, double coeff)` принимает именно индекс, а
  не тег.
- `System::calculateConstraintErrorByTag(tagId)` собирает все скалярные
  ограничения с данным тегом: если ограничение одно — возвращает его знаковую
  невязку, если несколько — RMS-агрегат.
- `tagId >= 0` участвует в диагностике; `tagId < 0` — временные ограничения,
  которые не влияют на диагностику (см. `SpecialTag::DefaultTemporaryConstraint
  = -1`).
- `tagId == 0` — высокоприоритетные ограничения; они исключаются из
  предлагаемых пользователю конфликтующих наборов.
- `internalAlignment = InternalAlignment` исключает служебные ограничения из
  пользовательских предложений по конфликтам.

### 1.2. `ConstraintType`

| Значение | Имя | Класс |
|---:|---|---|
| 0 | `None` | `Constraint` |
| 1 | `Equal` | `ConstraintEqual` |
| 2 | `Difference` | `ConstraintDifference` |
| 3 | `P2PDistance` | `ConstraintP2PDistance` |
| 4 | `P2PAngle` | `ConstraintP2PAngle` |
| 5 | `P2LDistance` | `ConstraintP2LDistance` |
| 6 | `PointOnLine` | `ConstraintPointOnLine` |
| 7 | `PointOnPerpBisector` | `ConstraintPointOnPerpBisector` |
| 8 | `Parallel` | `ConstraintParallel` |
| 9 | `Perpendicular` | `ConstraintPerpendicular` |
| 10 | `L2LAngle` | `ConstraintL2LAngle` |
| 11 | `MidpointOnLine` | `ConstraintMidpointOnLine` |
| 12 | `TangentCircumf` | `ConstraintTangentCircumf` |
| 13 | `PointOnEllipse` | `ConstraintPointOnEllipse` |
| 14 | `TangentEllipseLine` | `ConstraintEllipseTangentLine` |
| 15 | `InternalAlignmentPoint2Ellipse` | `ConstraintInternalAlignmentPoint2Ellipse` |
| 16 | `EqualMajorAxesConic` | `ConstraintEqualMajorAxesConic` |
| 17 | `EllipticalArcRangeToEndPoints` | не реализовано (legacy) |
| 18 | `AngleViaPoint` | `ConstraintAngleViaPoint` |
| 19 | `Snell` | `ConstraintSnell` |
| 20 | `CurveValue` | `ConstraintCurveValue` |
| 21 | `PointOnHyperbola` | `ConstraintPointOnHyperbola` |
| 22 | `InternalAlignmentPoint2Hyperbola` | `ConstraintInternalAlignmentPoint2Hyperbola` |
| 23 | `PointOnParabola` | `ConstraintPointOnParabola` |
| 24 | `EqualFocalDistance` | `ConstraintEqualFocalDistance` |
| 25 | `EqualLineLength` | `ConstraintEqualLineLength` |
| 26 | `CenterOfGravity` | `ConstraintCenterOfGravity` |
| 27 | `WeightedLinearCombination` | `ConstraintWeightedLinearCombination` |
| 28 | `SlopeAtBSplineKnot` | `ConstraintSlopeAtBSplineKnot` |
| 29 | `PointOnBSpline` | `ConstraintPointOnBSpline` |
| 30 | `C2CDistance` | `ConstraintC2CDistance` |
| 31 | `C2LDistance` | `ConstraintC2LDistance` |
| 32 | `P2CDistance` | `ConstraintP2CDistance` |
| 33 | `AngleViaPointAndParam` | `ConstraintAngleViaPointAndParam` |
| 34 | `AngleViaPointAndTwoParams` | `ConstraintAngleViaPointAndTwoParams` |
| 35 | `AngleViaTwoPoints` | `ConstraintAngleViaTwoPoints` |
| 36 | `ArcLength` | `ConstraintArcLength` |

## 2. Примитивы

Все геометрические объекты хранят указатели на параметры `double*`, поэтому
ограничение не копирует геометрию, а связывает её параметры.

| Примитив | C++ тип | Параметры | Комментарий |
|---|---|---|---|
| Точка | `Point` | `x`, `y` | два независимых параметра |
| Отрезок/прямая | `Line` | `p1`, `p2` | направление задаётся `p1 -> p2` |
| Окружность | `Circle` | `center`, `rad` | `rad` — отдельный параметр |
| Дуга | `Arc` | `center`, `rad`, `startAngle`, `endAngle`, `start`, `end` | `start`/`end` — точки; их привязка к углам делается `ArcRules` |
| Эллипс | `Ellipse` | `center`, `focus1`, `radmin` | большая полуось вычисляется: `a = sqrt(b^2 + abs(F1-C)^2)` |
| Дуга эллипса | `ArcOfEllipse` | параметры `Ellipse` + `startAngle`, `endAngle`, `start`, `end` | углы задают диапазон |
| Гипербола | `Hyperbola` | `center`, `focus1`, `radmin` | `a = sqrt(abs(F1-C)^2 - b^2)` |
| Дуга гиперболы | `ArcOfHyperbola` | параметры `Hyperbola` + углы и точки | |
| Парабола | `Parabola` | `vertex`, `focus1` | фокусное расстояние `f = abs(F-V)` |
| Дуга параболы | `ArcOfParabola` | параметры `Parabola` + углы и точки | |
| B-сплайн | `BSpline` | `poles`, `weights`, `knots`, `degree`, `mult`, `periodic`, `start`, `end` | NURBS; веса — отдельные параметры |

Важно: у дуг точки `start`/`end` не вычисляются автоматически. Чтобы они
соответствовали `startAngle`/`endAngle`, нужно добавить `ArcRules`
(или `ArcOfEllipseRules`, `ArcOfHyperbolaRules`, `ArcOfParabolaRules`).

## 3. Базовые скалярные ограничения

| Метод | Класс / тип | Параметры | Невязка |
|---|---|---|---|
| `addConstraintEqual` | `ConstraintEqual` / `Equal` | `param1`, `param2`, `tagId=0`, `driving=true`, `internalalignment` | `scale * (p1 - p2)` |
| `addConstraintProportional` | `ConstraintEqual` / `Equal` | `param1`, `param2`, `ratio`, `tagId`, `driving` | `scale * (p1 - ratio * p2)` |
| `addConstraintDifference` | `ConstraintDifference` / `Difference` | `param1`, `param2`, `difference`, `tagId=0`, `driving` | `scale * ((p2 - p1) - difference)` |
| прямое создание | `ConstraintCenterOfGravity` / `CenterOfGravity` | `q`, точки `p_i`, веса `w_i` | `scale * (q - sum(p_i * w_i))` |
| прямое создание | `ConstraintWeightedLinearCombination` / `WeightedLinearCombination` | точка, полюса, веса, факторы | `scale * (P * sum(w_i f_i) - sum(pole_i w_i f_i))` |

`addConstraintEqual` — основное «скалярное равенство». При `ratio = 1`
получается `p1 = p2`; при `ratio = 0.5` — `p1 = 0.5 * p2` (так сделаны
диаметры).

`ConstraintEqual::evaluate()` для driven-режима записывает `p2 = p1 / ratio`;
`ratio` не должен быть нулём.

`ConstraintDifference` задаёт `p2 - p1 = difference`.
`evaluate()` записывает измеренное значение `difference`.

`ConstraintCenterOfGravity` и `ConstraintWeightedLinearCombination` не имеют
отдельных фабричных методов в `System`; их можно добавить через
`System::addConstraint(new ConstraintCenterOfGravity(...))` или
`System::addConstraint(new ConstraintWeightedLinearCombination(...))`.
`WeightedLinearCombination` используется внутри
`addConstraintInternalAlignmentKnotPoint` для узлов B-сплайна.

## 4. Расстояния

| Метод | Класс / тип | Примитивы | Ключевые параметры |
|---|---|---|---|
| `addConstraintP2PDistance` | `ConstraintP2PDistance` / `P2PDistance` | две точки | `distance` |
| `addConstraintP2LDistance` | `ConstraintP2LDistance` / `P2LDistance` | точка, прямая | `distance`, `ccw` |
| `addConstraintC2CDistance` | `ConstraintC2CDistance` / `C2CDistance` | две окружности | `dist`, `c1bigger` |
| `addConstraintC2LDistance` | `ConstraintC2LDistance` / `C2LDistance` | окружность, прямая | `dist`, `ccw`, `internal` |
| `addConstraintP2CDistance` | `ConstraintP2CDistance` / `P2CDistance` | точка, окружность | `distance` |
| `addConstraintArcLength` | `ConstraintArcLength` / `ArcLength` | дуга | `dist` |

### 4.1. `addConstraintP2PDistance`

```cpp
int addConstraintP2PDistance(Point& p1, Point& p2,
                             double* distance,
                             int tagId = 0, bool driving = true);
```

Математика:

```text
e = scale * (sqrt((p1.x - p2.x)^2 + (p1.y - p2.y)^2) - distance)
```

То есть расстояние между точками равно `distance`. `distance >= 0`.
`evaluate()` записывает измеренное расстояние.

### 4.2. `addConstraintP2LDistance` (знаковое, с `ccw`)

```cpp
int addConstraintP2LDistance(Point& p, Line& l,
                             double* distance,
                             bool ccw,
                             int tagId = 0, bool driving = true);
```

Сначала вычисляется знаковое расстояние от точки до направленной прямой:

```text
d  = sqrt((p2.x - p1.x)^2 + (p2.y - p1.y)^2)          // длина прямой
A  = (p2.x - p1.x) * (p.y - p1.y)
   - (p2.y - p1.y) * (p.x - p1.x)                     // ориентированная площадь
h  = A / d                                            // знаковое расстояние
```

Затем:

```text
target = ccw ? +abs(distance) : -abs(distance)
e      = scale * (h - target)
```

- `ccw = true` — точка должна лежать на положительной стороне прямой
  `p1 -> p2` (слева, если смотреть вдоль направления);
- `ccw = false` — на отрицательной стороне.

Ограничение знаковое. Чтобы сохранить прежнее «расстояние по модулю»,
сторону выбирают по начальной геометрии. `evaluate()` записывает `abs(h)` в
`distance`.

### 4.3. `addConstraintC2CDistance`

```cpp
int addConstraintC2CDistance(Circle& c1, Circle& c2,
                             double* dist,
                             std::optional<bool> c1bigger,
                             int tagId, bool driving = true);
```

Класс `ConstraintC2CDistance` различает два случая по текущей геометрии.

Внешний случай — центры не находятся внутри другой окружности:

```text
L = |C1 - C2|
e = scale * (L - (r1 + r2 + dist))
```

То есть `dist` — зазор между внешними границами окружностей.

Внутренний случай — одна окружность находится внутри другой:

```text
r_big, r_small — больший и меньший радиусы
e = scale * (r_big - (r_small + L + dist))
```

То есть `dist` — зазор между внутренней и внешней окружностью.

`c1bigger`:

- `std::nullopt` — больший радиус выбирается сравнением `r1` и `r2`;
- `true` — считать `r1` большим;
- `false` — считать `r2` большим.

`evaluate()` записывает в `dist` соответствующую измеренную величину.

### 4.4. `addConstraintC2LDistance` (знаковое, с `ccw` и `internal`)

```cpp
int addConstraintC2LDistance(Circle& c, Line& l,
                             double* dist,
                             bool ccw, bool internal,
                             int tagId, bool driving = true);
```

Вычисляется знаковое расстояние от центра окружности до прямой:

```text
h = cross(l.p2 - l.p1, c.center - l.p1) / |l.p2 - l.p1|
```

Целевое расстояние:

```text
outer = r + abs(dist)       // снаружи окружности
inner = r - abs(dist)       // внутри окружности

target = internal ? inner : outer
target = ccw ? target : -target

e = scale * (target - h)
```

- `ccw` — сторона прямой, на которой находится центр окружности;
- `internal = false` — расстояние между прямой и внешней границей окружности
  (`h = r + dist`);
- `internal = true` — расстояние между прямой и внутренней границей
  (`h = r - dist`).

`evaluate()` записывает `abs(r - abs(h))` в `dist`.

### 4.5. `addConstraintP2CDistance`

```cpp
int addConstraintP2CDistance(Point& p, Circle& c,
                             double* distance,
                             int tagId = 0, bool driving = true);
```

Расстояние от точки до **границы** окружности (не до центра):

```text
L = |p - c.center|

если L >= r:
    e = scale * (r + distance - L)
иначе:
    e = scale * (r - distance - L)
```

То есть `distance` — неотрицательный зазор от точки до окружности.
`evaluate()` записывает `abs(L - r)`.

### 4.6. `addConstraintArcLength`

```cpp
int addConstraintArcLength(Arc& a, double* dist,
                           int tagId, bool driving = true);
```

Углы нормализуются в положительный CCW-диапазон:

```text
start' = startAngle, приведённый к >= 0
end'   = endAngle, приведённый к >= start'

e = scale * (rad * (end' - start') - dist)
```

Ограничение предполагает дугу, идущую против часовой стрелки.
`evaluate()` записывает `rad * (end' - start')`.

## 5. Углы

| Метод                                    | Класс / тип                                                         | Примитивы                        | Что задаёт                        |
| ---------------------------------------- | ------------------------------------------------------------------- | -------------------------------- | --------------------------------- |
| `addConstraintP2PAngle`                  | `ConstraintP2PAngle` / `P2PAngle`                                   | две точки                        | направление вектора `p1 -> p2`    |
| `addConstraintL2LAngle`                  | `ConstraintL2LAngle` / `L2LAngle`                                   | две прямые или 4 точки           | угол между направленными прямыми  |
| `addConstraintAngleViaPoint`             | `ConstraintAngleViaPoint` / `AngleViaPoint`                         | две кривые, точка                | угол нормалей в общей точке       |
| `addConstraintAngleViaTwoPoints`         | `ConstraintAngleViaTwoPoints` / `AngleViaTwoPoints`                 | две кривые, две точки            | угол нормалей в разных точках     |
| `addConstraintAngleViaPointAndParam`     | `ConstraintAngleViaPointAndParam` / `AngleViaPointAndParam`         | две кривые, точка, параметр      | нормаль первой кривой в параметре |
| `addConstraintAngleViaPointAndTwoParams` | `ConstraintAngleViaPointAndTwoParams` / `AngleViaPointAndTwoParams` | две кривые, точка, два параметра | нормали в двух параметрах         |
| `addConstraintSnellsLaw`                 | `ConstraintSnell` / `Snell`                                         | два луча, граница, точка         | закон Снеллиуса                   |

### 5.1. `addConstraintP2PAngle`

```cpp
int addConstraintP2PAngle(Point& p1, Point& p2, double* angle,
                          double incrAngle,
                          int tagId = 0, bool driving = true);

int addConstraintP2PAngle(Point& p1, Point& p2, double* angle,
                          int tagId = 0, bool driving = true);
```

Математика:

```text
dx = p2.x - p1.x
dy = p2.y - p1.y
e  = scale * wrap(atan2(dy, dx) - (angle + incrAngle))
```

То есть направление вектора `p1 -> p2` равно `angle + incrAngle`.
`incrAngle` используется в составных ограничениях (например, для
перпендикулярности дуге). `evaluate()` записывает `atan2(dy, dx) - incrAngle`.

Важно: во второй перегрузке (без `incrAngle`) текущий код игнорирует
переданный `tagId` и всегда создаёт ограничение с тегом `0`:
`return addConstraintP2PAngle(p1, p2, angle, 0., 0, driving);`.

### 5.2. `addConstraintL2LAngle`

```cpp
int addConstraintL2LAngle(Line& l1, Line& l2, double* angle,
                          int tagId = 0, bool driving = true);

int addConstraintL2LAngle(Point& l1p1, Point& l1p2,
                          Point& l2p1, Point& l2p2,
                          double* angle,
                          int tagId = 0, bool driving = true);
```

```text
v1 = l1.p2 - l1.p1
v2 = l2.p2 - l2.p1
e  = scale * wrap(atan2(v2) - atan2(v1) - angle)
```

То есть направленный угол от `v1` к `v2` равен `angle`.
`evaluate()` записывает измеренный угол.

### 5.3. Углы через нормали кривых

Все четыре варианта используют нормали `n1`, `n2` в заданных точках/параметрах:

```text
e = scale * wrap(atan2(n2) - atan2(n1) - angle)
```

Различия:

- `AngleViaPoint` — обе нормали в одной точке `p`;
- `AngleViaTwoPoints` — `n1` в `p1`, `n2` в `p2`;
- `AngleViaPointAndParam` — `n1` в параметре `cparam` первой кривой,
  `n2` в точке `p`;
- `AngleViaPointAndTwoParams` — `n1` в `cparam1`, `n2` в `cparam2`.

Подходят для любых `Curve`: прямых, окружностей, дуг, эллипсов, гипербол,
парабол, B-сплайнов.

### 5.4. `addConstraintSnellsLaw`

```cpp
int addConstraintSnellsLaw(Curve& ray1, Curve& ray2, Curve& boundary,
                           Point p,
                           double* n1, double* n2,
                           bool flipn1, bool flipn2,
                           int tagId, bool driving = true);
```

Вычисляются касательные направления лучей и границы в точке `p`, затем:

```text
sin1 = dot(tangent(ray1), tangent(boundary))
sin2 = dot(tangent(ray2), tangent(boundary))

если flipn1: sin1 = -sin1
если flipn2: sin2 = -sin2

e = scale * (n1 * sin1 - n2 * sin2)
```

То есть `n1 * sin(theta1) = n2 * sin(theta2)`; `flipn1`/`flipn2` меняют знак
синуса для выбора нужной ветви/ориентации.

## 6. Инцидентность, положение и кривые

| Метод                               | Класс / тип                                             | Примитивы                   | Что задаёт                               |
| ----------------------------------- | ------------------------------------------------------- | --------------------------- | ---------------------------------------- |
| `addConstraintP2PCoincident`        | 2 × `ConstraintEqual` / `Equal`                         | две точки                   | `x1 = x2`, `y1 = y2`                     |
| `addConstraintPointOnLine`          | `ConstraintPointOnLine` / `PointOnLine`                 | точка, прямая или две точки | точка лежит на прямой                    |
| `addConstraintPointOnPerpBisector`  | `ConstraintPointOnPerpBisector` / `PointOnPerpBisector` | точка, прямая или две точки | точка на серединном перпендикуляре       |
| `addConstraintMidpointOnLine`       | `ConstraintMidpointOnLine` / `MidpointOnLine`           | две прямые или 4 точки      | середина первого отрезка лежит на втором |
| `addConstraintPointOnCircle`        | `ConstraintP2PDistance` / `P2PDistance`                 | точка, окружность           | точка на окружности                      |
| `addConstraintPointOnArc`           | `ConstraintP2PDistance` / `P2PDistance`                 | точка, дуга                 | точка на полной окружности дуги          |
| `addConstraintPointOnEllipse`       | `ConstraintPointOnEllipse` / `PointOnEllipse`           | точка, эллипс/дуга эллипса  | точка на эллипсе                         |
| `addConstraintPointOnHyperbolicArc` | `ConstraintPointOnHyperbola` / `PointOnHyperbola`       | точка, дуга гиперболы       | точка на гиперболе                       |
| `addConstraintPointOnParabolicArc`  | `ConstraintPointOnParabola` / `PointOnParabola`         | точка, дуга параболы        | точка на параболе                        |
| `addConstraintPointOnBSpline`       | `ConstraintPointOnBSpline` / `PointOnBSpline`           | точка, B-сплайн, параметр   | точка на NURBS                           |
| `addConstraintCurveValue`           | `ConstraintCurveValue` / `CurveValue`                   | точка, кривая, параметр     | координата точки равна значению кривой   |
| `addConstraintArcRules`             | `ConstraintCurveValue` × 2                              | дуга                        | привязка `start/end` к углам             |

`addConstraintP2PCoincident` — производное ограничение: создаёт два
`ConstraintEqual` с одним тегом, `x1 = x2` и `y1 = y2`.

### 6.1. `PointOnLine`

```text
h = cross(l.p2 - l.p1, p - l.p1) / |l.p2 - l.p1|
e = scale * h
```

`h = 0` — точка на бесконечной прямой. Есть перегрузка с двумя точками
`lp1`, `lp2`: она строит ту же прямую из двух точек.

### 6.2. `PointOnPerpBisector`

Для точек `p0`, `p1`, `p2`:

```text
D = normalize(p2 - p1)
e = scale * (dot(p0 - p1, D) + dot(p0 - p2, D))
```

`e = 0` означает, что проекции `p0` на направление `p1 -> p2` симметричны,
то есть `p0` лежит на серединном перпендикуляре к `p1p2`.
Перегрузка с `Line` использует её `p1`, `p2`.

### 6.3. `MidpointOnLine`

```text
M = (l1.p1 + l1.p2) / 2
h = cross(l2.p2 - l2.p1, M - l2.p1) / |l2.p2 - l2.p1|
e = scale * h
```

Середина первого отрезка лежит на втором отрезке.

### 6.4. `PointOnCircle` и `PointOnArc`

Это производные ограничения:

```cpp
addConstraintPointOnCircle(p, c) == addConstraintP2PDistance(p, c.center, c.rad)
addConstraintPointOnArc(p, a)   == addConstraintP2PDistance(p, a.center, a.rad)
```

То есть `|p - center| = rad`. Для дуги это ограничивает полную окружность, но
не угловой диапазон; диапазон задаётся `ArcRules`.

### 6.5. `PointOnEllipse`

```text
C  = center
F1 = focus1
F2 = 2*C - F1
b  = radmin
a  = sqrt(b^2 + |F1 - C|^2)

e = scale * (|P - F1| + |P - F2| - 2*a)
```

Классическое свойство эллипса: сумма расстояний до фокусов равна `2a`.
Для `ArcOfEllipse` ограничение действует на полный эллипс; угловой диапазон
задаётся `ArcOfEllipseRules`.

### 6.6. `PointOnHyperbola`

```text
C  = center
F1 = focus1
F2 = 2*C - F1
b  = radmin
a  = sqrt(|F1 - C|^2 - b^2)

e = scale * (|P - F2| - |P - F1| - 2*a)
```

То есть разность расстояний до фокусов равна `2a` (одна ветвь гиперболы
выбирается знаком). Для `ArcOfHyperbola` ограничение действует на полную
гиперболу; угловой диапазон задаётся `ArcOfHyperbolaRules`.

### 6.7. `PointOnParabola`

```text
F     = focus1
V     = vertex
f     = |F - V|
xdir  = normalize(F - V)
proj  = dot(P - F, xdir)

e = scale * (|P - F| - 2*f - proj)
```

Это определение параболы через фокус и директрису: расстояние до фокуса равно
расстоянию до директрисы. Для `ArcOfParabola` ограничение действует на полную
параболу; угловой диапазон задаётся `ArcOfParabolaRules`.

### 6.8. `PointOnBSpline`

```cpp
int addConstraintPointOnBSpline(Point& p, BSpline& b,
                                double* pointparam,
                                int tagId, bool driving = true);
```

Добавляет **два** скалярных ограничения (для `x` и `y`). Для параметра `u`:

```text
W(u) = sum_i B_i(u) * w_i
X(u) = sum_i B_i(u) * w_i * pole_i.x
Y(u) = sum_i B_i(u) * w_i * pole_i.y

e_x = scale * (p.x * W(u) - X(u))
e_y = scale * (p.y * W(u) - Y(u))
```

`pointparam` — параметр `u`, который может быть как свободным, так и
фиксированным. Внутри выбирается нужный интервал узлов (`startpole`).

### 6.9. `CurveValue`

```cpp
int addConstraintCurveValue(Point& p, Curve& curve,
                            double* u,
                            int tagId = 0, bool driving = true);
```

Добавляет два ограничения:

```text
e_x = scale * (p.x - curve.Value(u).x)
e_y = scale * (p.y - curve.Value(u).y)
```

Используется для привязки концов дуг к параметрическим углам.

### 6.10. `ArcRules`

`addConstraintArcRules(a)` добавляет два `CurveValue`:

```text
a.start = a.Value(a.startAngle)
a.end   = a.Value(a.endAngle)
```

Аналогично `addConstraintArcOfEllipseRules`, `addConstraintArcOfHyperbolaRules`,
`addConstraintArcOfParabolaRules`.

Без этих ограничений `start`/`end` дуги — независимые точки и не обязаны
соответствовать её углам.

## 7. Ориентация, параллельность и перпендикулярность

| Метод | Класс / тип | Примитивы | Что задаёт |
|---|---|---|---|
| `addConstraintParallel` | `ConstraintParallel` / `Parallel` | две прямые | параллельность |
| `addConstraintPerpendicular` | `ConstraintPerpendicular` / `Perpendicular` | две прямые, 4 точки или 2 точки + `Line` | перпендикулярность |
| `addConstraintHorizontal` | `ConstraintEqual` / `Equal` | прямая или две точки | `y1 = y2` |
| `addConstraintVertical` | `ConstraintEqual` / `Equal` | прямая или две точки | `x1 = x2` |
| `addConstraintCoordinateX` | `ConstraintEqual` / `Equal` | точка, параметр | `p.x = x` |
| `addConstraintCoordinateY` | `ConstraintEqual` / `Equal` | точка, параметр | `p.y = y` |
| `addConstraintPerpendicularLine2Arc` | составное | прямая, дуга | прямая — нормаль к дуге в её начале |
| `addConstraintPerpendicularArc2Line` | составное | дуга, прямая | прямая — нормаль к дуге в её конце |
| `addConstraintPerpendicularCircle2Arc` | составное | окружность, дуга | окружность и дуга ортогональны в начале дуги |
| `addConstraintPerpendicularArc2Circle` | составное | дуга, окружность | окружность и дуга ортогональны в конце дуги |
| `addConstraintPerpendicularArc2Arc` | составное | две дуги | дуги ортогональны в общей точке |

### 7.1. `Parallel`

```text
v1 = l1.p2 - l1.p1
v2 = l2.p2 - l2.p1
scale = coef / (|v1| * |v2|)
e = scale * cross(v1, v2)
```

После нормировки `e = sin(angle)`; `e = 0` — прямые параллельны или
антипараллельны.

### 7.2. `Perpendicular`

```text
v1 = l1.p2 - l1.p1
v2 = l2.p2 - l2.p1
scale = coef / (|v1| * |v2|)
e = scale * dot(v1, v2)
```

После нормировки `e = cos(angle)`; `e = 0` — прямые перпендикулярны.
Есть перегрузки: две `Line`, четыре `Point`, `Point/Point/Line`.

### 7.3. `Horizontal`, `Vertical`, `CoordinateX`, `CoordinateY`

Это просто `ConstraintEqual`:

```text
Horizontal(line):     l.p1.y - l.p2.y = 0
Vertical(line):       l.p1.x - l.p2.x = 0
CoordinateX(p, x):    p.x - x = 0
CoordinateY(p, y):    p.y - y = 0
```

### 7.4. `PerpendicularLine2Arc`

Добавляет:

1. `p2` совпадает с `a.start`;
2. направление `p1 -> p2` равно `a.startAngle` (с добавкой `pi`, если нужно
   согласовать ориентацию).

Поскольку `startAngle` — радиальное направление, прямая оказывается
**нормалью** к дуге в её начале (перпендикулярна касательной), а не касательной.

`PerpendicularArc2Line` — зеркальный вариант для `a.end`.

### 7.5. `PerpendicularCircle2Arc`

Добавляет:

1. `a.start` лежит на окружности `(center, radius)`;
2. направление `center -> a.start` совпадает с касательным направлением дуги
   в начале (`a.startAngle ± pi/2`).

Так окружность и дуга пересекаются ортогонально.
`PerpendicularArc2Circle` — вариант для `a.end`.

### 7.6. `PerpendicularArc2Arc`

Добавляет совпадение выбранных концов дуг и
`Perpendicular(center1, p1, center2, p2)`: радиусы в общей точке
перпендикулярны, значит дуги ортогональны. Параметры `reverse1`, `reverse2`
выбирают, использовать `start` или `end` каждой дуги.

## 8. Касания

| Метод | Класс / тип | Примитивы | Ключевые параметры |
|---|---|---|---|
| `addConstraintTangentCircumf` | `ConstraintTangentCircumf` / `TangentCircumf` | две окружности (центры + радиусы) | `internal` |
| `addConstraintTangent(Line, Circle)` | `P2LDistance` | прямая, окружность | `ccw` |
| `addConstraintTangent(Line, Arc)` | `P2LDistance` | прямая, дуга | `ccw` |
| `addConstraintTangent(Line, Ellipse)` | `ConstraintEllipseTangentLine` / `TangentEllipseLine` | прямая, эллипс | — |
| `addConstraintTangent(Circle, Circle)` | `TangentCircumf` | две окружности | авто `internal` |
| `addConstraintTangent(Arc, Arc)` | `TangentCircumf` | две дуги | авто `internal` |
| `addConstraintTangent(Circle, Arc)` | `TangentCircumf` | окружность, дуга | авто `internal` |
| `addConstraintTangentAtBSplineKnot` | `ConstraintSlopeAtBSplineKnot` / `SlopeAtBSplineKnot` | B-сплайн, прямая | `knotindex` |

### 8.1. `TangentCircumf`

```cpp
int addConstraintTangentCircumf(Point& p1, Point& p2,
                                double* rad1, double* rad2,
                                bool internal = false,
                                int tagId = 0, bool driving = true);
```

Внешнее касание (`internal = false`):

```text
e = scale * (|C1 - C2|^2 - (r1 + r2)^2)
```

Внутреннее касание (`internal = true`):

```text
e = scale * (|C1 - C2|^2 - (r1 - r2)^2)
```

При почти концентрических окружностях используется устойчивая замена
`e = scale * (r1 - r2)`.

### 8.2. `addConstraintTangent(Line, Circle/Arc, ccw)`

Прямая касается окружности/дуги тогда и только тогда, когда центр находится
на расстоянии радиуса от прямой:

```cpp
addConstraintTangent(l, c, ccw)
    == addConstraintP2LDistance(c.center, l, c.rad, ccw)

addConstraintTangent(l, a, ccw)
    == addConstraintP2LDistance(a.center, l, a.rad, ccw)
```

`ccw` выбирает сторону прямой, на которой находится центр. Это та самая
ccw-перегрузка, появившаяся в актуальном FreeCAD.

### 8.3. `addConstraintTangent(Line, Ellipse)`

Использует точное геометрическое условие касания эллипса и прямой:

1. фокус `F1` отражается относительно прямой в `F1'`;
2. для касательной расстояние `|F1' - F2|` равно `2a`, где `F2 = 2C - F1`,
   `a` — большая полуось.

```text
e = scale * (|F1' - F2| - 2*a)
```

Это ограничение не имеет `ccw`: сторона касания определяется начальной
геометрией и самим решением.

### 8.4. `addConstraintTangent` для окружностей и дуг

Вызывает `TangentCircumf`, автоматически определяя `internal` по текущей
геометрии:

```text
internal = (|C1 - C2| < r1) || (|C1 - C2| < r2)
```

То есть если центры находятся внутри окружностей — выбирается внутреннее
касание, иначе внешнее. Если начальная геометрия неоднозначна, используйте
`addConstraintTangentCircumf` с явным `internal`.

### 8.5. `addConstraintTangentAtBSplineKnot`

```cpp
int addConstraintTangentAtBSplineKnot(BSpline& b, Line& l,
                                      unsigned int knotindex,
                                      int tagId = 0, bool driving = true);
```

Строит касательную B-сплайна в узле `knotindex` как линейную комбинацию
ближайших полюсов и требует её коллинеарности прямой:

```text
e = scale * cross(tangent_bspline(knot), direction_line)
```

Нормировка делает невязку безразмерной.

## 9. Равенства, радиусы и симметрия

| Метод | Класс / тип | Примитивы | Невязка |
|---|---|---|---|
| `addConstraintCircleRadius` | `ConstraintEqual` / `Equal` | окружность, параметр | `rad - radius` |
| `addConstraintArcRadius` | `ConstraintEqual` / `Equal` | дуга, параметр | `rad - radius` |
| `addConstraintCircleDiameter` | `ConstraintEqual` / `Equal` (ratio 0.5) | окружность, параметр | `rad - 0.5 * diameter` |
| `addConstraintArcDiameter` | `ConstraintEqual` / `Equal` (ratio 0.5) | дуга, параметр | `rad - 0.5 * diameter` |
| `addConstraintEqualLength` | `ConstraintEqualLineLength` / `EqualLineLength` | две прямые | `scale * (abs(l2) - abs(l1))` |
| `addConstraintEqualRadius(Circle,Circle)` | `ConstraintEqual` / `Equal` | две окружности | `r1 - r2` |
| `addConstraintEqualRadius(Circle,Arc)` | `ConstraintEqual` / `Equal` | окружность, дуга | `r1 - r2` |
| `addConstraintEqualRadius(Arc,Arc)` | `ConstraintEqual` / `Equal` | две дуги | `r1 - r2` |
| `addConstraintEqualRadii(Ellipse,Ellipse)` | `Equal` + `EqualMajorAxesConic` | два эллипса | `b1-b2=0`, `a1-a2=0` |
| `addConstraintEqualRadii(ArcOfHyperbola,ArcOfHyperbola)` | `Equal` + `EqualMajorAxesConic` | две гиперболы | `b1-b2=0`, `a1-a2=0` |
| `addConstraintEqualFocus(ArcOfParabola,ArcOfParabola)` | `ConstraintEqualFocalDistance` / `EqualFocalDistance` | две параболы | `f2 - f1 = 0` |
| `addConstraintP2PSymmetric(p1,p2,Line)` | `Perpendicular` + `MidpointOnLine` | две точки, прямая | зеркальная симметрия |
| `addConstraintP2PSymmetric(p1,p2,Point)` | `PointOnPerpBisector` + `PointOnLine` | три точки | `p` — середина `p1p2` |

`EqualMajorAxesConic` — отдельный класс для сравнения больших полуосей
конических сечений:

```text
e = scale * (a2 - a1)
```

`EqualFocalDistance`:

```text
f1 = |vertex1 - focus1|
f2 = |vertex2 - focus2|
e  = scale * (f2 - f1)
```

`P2PSymmetric(p1, p2, Line)`:

1. `Perpendicular(p1, p2, line)`;
2. `MidpointOnLine(p1, p2, line)`.

Вместе это означает, что `p1` и `p2` — зеркальные отражения относительно
прямой.

`P2PSymmetric(p1, p2, Point p)`:

1. `p` лежит на серединном перпендикуляре к `p1p2`;
2. `p` лежит на прямой `p1p2`.

Вместе это означает, что `p` — середина отрезка `p1p2` (точечная симметрия).

## 10. Internal alignment (оси, фокусы, B-сплайны)

Эти ограничения помечены `internalAlignment = InternalAlignment` и не
предлагаются пользователю как конфликтующие. Они связывают служебные точки
(концы осей, фокусы, контрольные точки) с параметрами конических сечений и
B-сплайнов.

| Метод | Примитивы | Назначение |
|---|---|---|
| `addConstraintInternalAlignmentPoint2Ellipse` | точка, эллипс, `InternalAlignmentType` | привязать координату точки к оси/фокусу эллипса |
| `addConstraintInternalAlignmentEllipseMajorDiameter` | две точки, эллипс | обе точки — концы большой оси |
| `addConstraintInternalAlignmentEllipseMinorDiameter` | две точки, эллипс | обе точки — концы малой оси |
| `addConstraintInternalAlignmentEllipseFocus1` | точка, эллипс | точка совпадает с `focus1` |
| `addConstraintInternalAlignmentEllipseFocus2` | точка, эллипс | точка совпадает со вторым фокусом |
| `addConstraintInternalAlignmentPoint2Hyperbola` | точка, гипербола, `InternalAlignmentType` | привязать координату точки к оси гиперболы |
| `addConstraintInternalAlignmentHyperbolaMajorDiameter` | две точки, гипербола | концы большой оси |
| `addConstraintInternalAlignmentHyperbolaMinorDiameter` | две точки, гипербола | концы малой оси |
| `addConstraintInternalAlignmentHyperbolaFocus` | точка, гипербола | `focus1` |
| `addConstraintInternalAlignmentParabolaFocus` | точка, парабола | `focus1` |
| `addConstraintInternalAlignmentBSplineControlPoint` | B-сплайн, окружность, `poleindex` | полюс/вес контрольной точки |
| `addConstraintInternalAlignmentKnotPoint` | B-сплайн, точка, `knotindex` | точка узла B-сплайна |

### 10.1. `InternalAlignmentType`

| Значение | Имя | Куда привязывается |
|---:|---|---|
| 0 | `EllipsePositiveMajorX` | `+a` по большой оси, координата `x` |
| 1 | `EllipsePositiveMajorY` | `+a` по большой оси, координата `y` |
| 2 | `EllipseNegativeMajorX` | `-a` по большой оси, `x` |
| 3 | `EllipseNegativeMajorY` | `-a` по большой оси, `y` |
| 4 | `EllipsePositiveMinorX` | `+b` по малой оси, `x` |
| 5 | `EllipsePositiveMinorY` | `+b` по малой оси, `y` |
| 6 | `EllipseNegativeMinorX` | `-b` по малой оси, `x` |
| 7 | `EllipseNegativeMinorY` | `-b` по малой оси, `y` |
| 8 | `EllipseFocus2X` | второй фокус, `x` |
| 9 | `EllipseFocus2Y` | второй фокус, `y` |
| 10 | `HyperbolaPositiveMajorX` | `+a` по большой оси, `x` |
| 11 | `HyperbolaPositiveMajorY` | `+a` по большой оси, `y` |
| 12 | `HyperbolaNegativeMajorX` | `-a` по большой оси, `x` |
| 13 | `HyperbolaNegativeMajorY` | `-a` по большой оси, `y` |
| 14 | `HyperbolaPositiveMinorX` | `+b` от положительной вершины, `x` |
| 15 | `HyperbolaPositiveMinorY` | `+b` от положительной вершины, `y` |
| 16 | `HyperbolaNegativeMinorX` | `-b` от положительной вершины, `x` |
| 17 | `HyperbolaNegativeMinorY` | `-b` от положительной вершины, `y` |

### 10.2. Как считается точка привязки

Для эллипса:

```text
emaj = normalize(F1 - C)
emin = rotate90ccw(emaj)
a    = sqrt(b^2 + |F1 - C|^2)

PositiveMajor: C + emaj * a
NegativeMajor: C - emaj * a
PositiveMinor: C + emin * b
NegativeMinor: C - emin * b
Focus2:        2*C - F1
```

Для гиперболы:

```text
emaj = normalize(F1 - C)
emin = rotate90ccw(emaj)
a    = sqrt(|F1 - C|^2 - b^2)

PositiveMajor: C + emaj * a
NegativeMajor: C - emaj * a
PositiveMinor: (C + emaj * a) + emin * b
NegativeMinor: (C + emaj * a) - emin * b
```

`InternalAlignmentPoint2Ellipse`/`...Hyperbola` добавляет **одно** скалярное
ограничение на координату `x` или `y`:

```text
e = point.coordinate - target.coordinate
```

Поэтому для полной привязки точки к концу оси вызываются два ограничения
(по `x` и по `y`).

`EllipseMajorDiameter` и `EllipseMinorDiameter` автоматически определяют,
какая из двух точек ближе к положительному концу оси, и создают 4 ограничения
(2 точки × 2 координаты). Аналогично устроены гиперболические варианты.

`InternalAlignmentBSplineControlPoint`:

```text
pole.x = circle.center.x
pole.y = circle.center.y
weight = circle.rad
```

`InternalAlignmentKnotPoint` использует `ConstraintWeightedLinearCombination`:
для узла `knotindex` берутся ближайшие полюса, вычисляются весовые факторы
`f_i`, и добавляются ограничения по `x` и `y`:

```text
P * sum(w_i * f_i) = sum(pole_i * w_i * f_i)
```

Для периодического B-сплайна узел 0 дополнительно совмещается с `start` и
`end`.

## 11. Настройки `System`, алгоритмы и диагностика

`GCS::System` хранит ограничения, неизвестные параметры, разбиение на
независимые подсистемы и настройки решателя.

### 11.1. Алгоритмы

```cpp
enum Algorithm { BFGS, LevenbergMarquardt, DogLeg };
enum QRAlgorithm { EigenDenseQR, EigenSparseQR };
enum DogLegGaussStep { FullPivLU, LeastNormFullPivLU, LeastNormLdlt };
enum DebugMode { NoDebug, Minimal, IterationLevel };
enum class SolveStatus {
    Success = 0,
    Converged = 1,
    Failed = 2,
    SuccessfulSolutionInvalid = 3
};
```

`SolveStatus::SuccessfulSolutionInvalid` в самом PlaneGCS не генерируется:
он предназначен для внешней проверки геометрии (например, OCC).

### 11.2. Основные поля `System`

| Поле | По умолчанию | Смысл |
|---|---:|---|
| `maxIter` | `100` | максимум итераций |
| `maxIterRedundant` | `100` | максимум итераций при redundant solving |
| `sketchSizeMultiplier` | `false` | умножать лимит итераций на размер системы |
| `convergence` | `1e-10` | порог сходимости |
| `convergenceRedundant` | `1e-10` | порог сходимости для redundant solving |
| `qrAlgorithm` | `EigenSparseQR` | QR-разложение для диагностики |
| `autoChooseAlgorithm` | `true` | автоматически выбирать dense/sparse QR |
| `autoQRThreshold` | `1000` | порог переключения |
| `dogLegGaussStep` | `FullPivLU` | метод шага Гаусса в DogLeg |
| `qrpivotThreshold` | `1e-13` | порог пивота QR |
| `debugMode` | `Minimal` | уровень отладочного вывода |
| `LM_eps`, `LM_eps1`, `LM_tau` | `1e-10`, `1e-80`, `1e-3` | параметры Levenberg–Marquardt |
| `DL_tolg`, `DL_tolx`, `DL_tolf` | `1e-80`, `1e-80`, `1e-10` | допуски DogLeg |
| `...Redundant` | аналогично | те же параметры для redundant solving |

### 11.3. Жизненный цикл

```cpp
void declareUnknowns(VEC_pD& params);
void declareDrivenParams(VEC_pD& params);
void initSolution(Algorithm alg = DogLeg);
SolveStatus solve(Algorithm alg = DogLeg, bool isRedundantsolving = false);
SolveStatus solve(VEC_pD& params, Algorithm alg = DogLeg, bool isRedundantsolving = false);
void applySolution();
void evaluateDrivenConstraints();
void undoSolution();
```

- `declareUnknowns` — список параметров, которые solver может менять.
- `declareDrivenParams` — параметры driven-ограничений.
- `initSolution` — сохраняет reference-конфигурацию, делает диагностику и
  разбивает систему на компоненты.
- `solve` — решает; `isRedundantsolving` включает специальный режим для
  избыточных систем.
- `applySolution` — записывает решение в параметры.
- `evaluateDrivenConstraints` — вычисляет значения driven-ограничений.

### 11.4. Диагностика

```cpp
int diagnose(Algorithm alg = DogLeg);
int diagnoseFull();              // локальное расширение этого vendored-снимка
int dofsNumber() const;
void getConflicting(VEC_I& out) const;
void getRedundant(VEC_I& out) const;
void getPartiallyRedundant(VEC_I& out) const;
void getDependentParams(VEC_pD& out) const;
bool isEmptyDiagnoseMatrix() const;
```

- `diagnose` — штатная диагностика PlaneGCS: степени свободы, конфликтующие,
  избыточные и частично избыточные теги.
- `diagnoseFull` — добавлен в этом репозитории: строит полную матрицу
  Якоби по driving-ограничениям, считает её ранг через SVD и определяет
  избыточные теги перебором строк. Используется native-обёрткой для
  `pgcs_diagnose_full`.
- `dofsNumber()` возвращает `-1`, если диагностика ещё не выполнена.

## 12. Native C API и AutoConstraint

`planegcs_native` предоставляет плоский C ABI, а `Planegcs.Native` —
C#-обёртку. Поддерживается подмножество ограничений PlaneGCS:

| C++ метод | C API | C# |
|---|---|---|
| `addConstraintP2PCoincident` | `pgcs_add_coincident` | `AddCoincident` |
| `addConstraintP2PDistance` | `pgcs_add_p2p_distance` | `AddP2PDistance` |
| `addConstraintP2LDistance` | `pgcs_add_p2l_distance` | `AddP2LDistance` |
| `addConstraintP2CDistance` | `pgcs_add_p2c_distance` | `AddP2CDistance` |
| `addConstraintC2CDistance` | `pgcs_add_c2c_distance` | `AddC2CDistance` |
| `addConstraintC2LDistance` | `pgcs_add_c2l_distance` | `AddC2LDistance` |
| `addConstraintL2LAngle` | `pgcs_add_l2l_angle` | `AddL2LAngle` |
| `addConstraintParallel` | `pgcs_add_parallel` | `AddParallel` |
| `addConstraintPerpendicular` | `pgcs_add_perpendicular` | `AddPerpendicular` |
| `addConstraintPointOnLine` | `pgcs_add_point_on_line` | `AddPointOnLine` |
| `addConstraintPointOnCircle` | `pgcs_add_point_on_circle` | `AddPointOnCircle` |
| `addConstraintPointOnArc` | `pgcs_add_point_on_arc` | `AddPointOnArc` |
| `addConstraintArcRules` | `pgcs_add_arc_rules` | `AddArcRules` |
| `addConstraintEqual` | `pgcs_add_equal` | `AddEqual` |
| `addConstraintEqualLength` | `pgcs_add_equal_length` | `AddEqualLength` |
| `addConstraintEqualRadius` | `pgcs_add_equal_radius` | `AddEqualRadius` |
| `addConstraintCircleRadius` | `pgcs_add_circle_radius` | `AddCircleRadius` |
| `addConstraintArcRadius` | `pgcs_add_arc_radius` | `AddArcRadius` |
| `addConstraintCircleDiameter` | `pgcs_add_circle_diameter` | `AddCircleDiameter` |
| `addConstraintArcDiameter` | `pgcs_add_arc_diameter` | `AddArcDiameter` |
| `addConstraintDifference` | `pgcs_add_difference` | `AddDifference` |
| `addConstraintP2PSymmetric` (line) | `pgcs_add_symmetric_line` | `AddSymmetricLine` |
| `addConstraintP2PSymmetric` (point) | `pgcs_add_symmetric_point` | `AddSymmetricPoint` |
| `addConstraintTangent(Line,Circle,ccw)` | `pgcs_add_tangent_line_circle` | `AddTangentLineCircle` |
| `addConstraintTangent(Line,Arc,ccw)` | `pgcs_add_tangent_line_arc` | `AddTangentLineArc` |
| `addConstraintTangent(Circle,Circle)` | `pgcs_add_tangent_circle_circle` | `AddTangentCircleCircle` |
| `addConstraintTangent(Circle,Arc)` | `pgcs_add_tangent_circle_arc` | `AddTangentCircleArc` |
| `addConstraintTangent(Arc,Arc)` | `pgcs_add_tangent_arc_arc` | `AddTangentArcArc` |

Особенности native-обёртки:

- `tag` должен быть `>= 0`; временные отрицательные теги не поддерживаются.
- `scale` должен быть `> 0`.
- C API возвращает переданный `tag`, а не внутренний индекс ограничения.
- Для `pgcs_add_p2l_distance`, `pgcs_add_c2l_distance` и line-circle/line-arc
  tangency добавлен параметр `ccw`.
- Для `pgcs_add_c2l_distance` добавлен параметр `internal`.
- `PlanegcsConstraintMapper` в AutoConstraint вычисляет `ccw` по начальной
  геометрии, чтобы сохранить прежнее поведение ограничений «по модулю».

## 13. Быстрые правила выбора

- Нужно расстояние между точками — `addConstraintP2PDistance`.
- Нужно расстояние от точки до отрезка/оси — `addConstraintP2LDistance` с
  `ccw`, выбранным по начальной стороне.
- Нужно расстояние от дуги/окружности до отрезка/оси —
  `addConstraintC2LDistance` с `ccw` и `internal = false` для внешнего зазора.
- Нужен зазор между двумя окружностями — `addConstraintC2CDistance`.
- Нужно касание прямой и окружности/дуги — `addConstraintTangent(l, c/a, ccw)`.
- Нужно касание двух окружностей/дуг — `addConstraintTangent(c1, c2)` или
  `addConstraintTangentCircumf` с явным `internal`.
- Нужен угол между прямыми — `addConstraintL2LAngle`.
- Нужен угол между произвольными кривыми — `AngleVia*`.
- Нужна точка на дуге с учётом её углового диапазона — `PointOnArc` +
  `ArcRules`.
- Нужен эллипс/гипербола/парабола с корректными концами — соответствующее
  `...Rules` + `PointOn...`.
- Нужны оси/фокусы конического сечения — `InternalAlignment*`.
- Нужен узел или контрольная точка B-сплайна — `InternalAlignmentKnotPoint` /
  `InternalAlignmentBSplineControlPoint`.

## 14. Ссылки на исходники

- `planegcs/Constraints.h` — объявления классов, `ConstraintType`,
  `InternalAlignmentType`.
- `planegcs/Constraints.cpp` — невязки, градиенты, `evaluate()`.
- `planegcs/GCS.h` — публичный API `System`, настройки, `SolveStatus`.
- `planegcs/GCS.cpp` — фабричные методы (`addConstraint*`), сборка ограничений,
  диагностика.
- `planegcs/Geo.h` — примитивы `Point`, `Line`, `Circle`, `Arc`, `Ellipse`,
  `Hyperbola`, `Parabola`, `BSpline`.
- `native/include/planegcs_native.h` — C ABI.
- `C:\vas\topic\RE-4-regulatization\src\Integrations\Planegcs.Native` — C#-обёртка
  AutoConstraint.
