////////////////////////////////////////
//
//	File : ai.c
//	CoSpace Robot
//	Version 1.0.0
//	Jan 1 2016
//	Copyright (C) 2016 CoSpace Robot. All Rights Reserved
//
//////////////////////////////////////
//
// ONLY C Code can be compiled.
//
/////////////////////////////////////

#pragma clang diagnostic push
#pragma ide diagnostic ignored "UnusedImportStatement"
#pragma ide diagnostic ignored "OCUnusedMacroInspection"
#pragma ide diagnostic ignored "OCUnusedGlobalDeclarationInspection"
#define CsBot_AI_H//DO NOT delete this line
#ifndef CSBOT_REAL

#include <windows.h>

#endif

#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include <stdlib.h>
#include <float.h>

#define DLL_EXPORT extern __declspec(dllexport)

#define false 0
#define true 1
#define NONE (-1)
#define E_A 0
#define E_B 1
#define E_C 3

#define CsBot_AI_C //DO NOT delete this line

typedef int bool;

//The robot ID : It must be two char, such as '00','kl' or 'Cr'.
char AI_MyID[2] = {'1', '0'};

double toRange(double value, double mmin, double mmax);

double direction_normalize(double value);

/***
 *    ######## ##    ## ########  ########  ######
 *       ##     ##  ##  ##     ## ##       ##    ##
 *       ##      ####   ##     ## ##       ##
 *       ##       ##    ########  ######    ######
 *       ##       ##    ##        ##             ##
 *       ##       ##    ##        ##       ##    ##
 *       ##       ##    ##        ########  ######
 */

typedef struct {
    double x, y;
} Vector;

Vector new_vector(double xa, double ya) {
    Vector v;
    v.x = xa;
    v.y = ya;
    return v;
}

typedef struct {
    int xa, ya, xb, yb;
} Area;

Area new_area(int xa, int ya, int xb, int yb) {
    Area v;
    v.xa = min(xa, xb);
    v.ya = min(ya, yb);
    v.xb = max(xa, xb);
    v.yb = max(ya, yb);
    return v;
}

typedef double Direction;

Direction new_Direction(double value) {
    return value;
}

typedef struct {
    Vector point;
    double radius;
} Anchor;

Anchor new_Anchor(Vector point, double radius) {
    Anchor v;
    v.point = point;
    v.radius = radius;
    return v;
}

typedef struct {
    Vector point;
    double radius;
    Direction direction;
} FlowPoint;

FlowPoint new_FlowPoint(Vector point, double radius, Direction direction) {
    FlowPoint v;
    v.point = point;
    v.radius = radius;
    v.direction = direction;
    return v;
}

typedef struct {
    Anchor pa;
    Anchor pb;
} FlowLine;

FlowLine new_FlowLine(Anchor pa, Anchor pb) {
    FlowLine v;
    v.pa = pa;
    v.pb = pb;
    return v;
}

typedef struct {
    int count;
    Vector points[30];
} Route;

typedef struct {
    bool withEnd;
    int count;
    Anchor points[25];
} FlowRoute;

typedef struct {
    double randomnessSize;
    int anchorCount;
    Anchor anchors[20];
    FlowLine flowLines[20];
    int flowPointCount;
    FlowPoint flowPoints[50];
    int routeCount;
    FlowRoute flowRoutes[15];
} Environment;

typedef struct {
    Vector a;
    Vector b;
} Wall;

Wall new_Wall(Vector a, Vector b) {
    Wall v;
    v.a = a;
    v.b = b;
    return v;
};

typedef struct {
    Area area;
    Area entry;
    Route route;
} SuperobjectRule;

typedef struct {
    FlowPoint point;
    FlowLine line;
} FlowPointAndLine;

FlowPointAndLine new_FlowPointAndLine(FlowPoint point, FlowLine line) {
    FlowPointAndLine o;
    o.point = point;
    o.line = line;
    return o;
}

//========== VECTOR ==========

Vector vector_radial(const Direction direction, double size) {
    return new_vector(
            cos(direction) * size,
            sin(direction) * size
    );
}

double vector_size(Vector A) {
    return sqrt(pow(A.x, 2) + pow(A.y, 2));
}

Vector vector_vectorTo(const Vector A, const Vector B) {
    return new_vector(B.x - A.x, B.y - A.y);
}

double vector_distanceTo(const Vector A, const Vector B) {
    return vector_size(vector_vectorTo(A, B));
}

Direction vector_direction(Vector A) {
    return new_Direction(atan2(A.y, A.x));
}

Direction vector_directionTo(const Vector A, Vector B) {
    return vector_direction(vector_vectorTo(A, B));
}

Vector vector_plus(Vector A, Vector B) {
    if (B.x == 0 && B.y == 0) return A;
    return new_vector(A.x + B.x, A.y + B.y);
}

Vector vector_minus(Vector A, Vector B) {
    return new_vector(A.x - B.x, A.y - B.y);
}

Vector vector_multiply(Vector A, double k) {
    return new_vector(A.x * k, A.y * k);
}

Vector vector_invert(Vector A) {
    return new_vector(-A.x, -A.y);
}

//========== AREA ==========

//========== DIRECTION ==========

Direction direction_fromDegrees(double degrees) {
    return new_Direction(degrees * M_PI / 180);
}

double toDegrees(double angle) {
    return angle * 180 / M_PI;
}

double direction_normalize(double value) {
    while (value < 0) value += 2 * M_PI;
    while (value >= 2 * M_PI) value -= 2 * M_PI;
    return value;
}

double direction_differenceTo(const Direction t, Direction target) {
    if (target < t) target += 2 * M_PI;
    return target - t;
}

double direction_difference(const Direction t, const Direction direction) {
    return min(
            direction_differenceTo(t, direction),
            direction_differenceTo(direction, t)
    );
}

Direction direction_plus(const Direction t, const Direction direction) {
    return new_Direction(t + direction);
}

Direction direction_plus_double(const Direction t, double val) {
    return new_Direction(t + val);
}

Direction direction_minus(const Direction t, const Direction direction) {
    return new_Direction(t - direction);
}

Direction direction_invert(const Direction t) {
    return new_Direction(t + M_PI);
}

double direction_degrees(const Direction t) {
    return t * 180 / M_PI;
}

Direction direction_mirrorWith(const Direction t, const Direction axis) {
    return new_Direction(2 * axis - t);
}

Direction direction_weightedAverageWith(Direction t, Direction direction, double weight) {
    return vector_direction(vector_plus(vector_radial(t, 1 - weight), vector_radial(direction, weight)));
}

Direction direction_averageWith(Direction t, Direction direction) {
    return direction_weightedAverageWith(t, direction, 0.5);
}

//========== ANCHOR ==========

//========== FLOWPOINT ==========

//========== FLOWLINE ==========

FlowPoint nearestFlowPointOnFlowLine(FlowLine t, Vector point) {
    Vector aToP = vector_vectorTo(t.pa.point, point);
    Vector aToB = vector_vectorTo(t.pa.point, t.pb.point);
    double atb2 = pow(aToB.x, 2) + pow(aToB.y, 2);
    double atp_dot_atb = aToP.x * aToB.x + aToP.y * aToB.y;

    double tx = toRange(atp_dot_atb / atb2, 0, 1);

    return new_FlowPoint(vector_plus(t.pa.point, vector_multiply(aToB, tx)),
                         tx * t.pb.radius + (1 - tx) * t.pa.radius,
                         vector_direction(aToB));
}

/*Vector nearestPointOnLine(Vector pa, Vector pb, Vector point) {
    Vector aToP = vector_vectorTo(pa, point);
    Vector aToB = vector_vectorTo(pa, pb);
    double atb2 = pow(aToB.x, 2) + pow(aToB.y, 2);
    double atp_dot_atb = aToP.x * aToB.x + aToP.y * aToB.y;

    double tx = toRange(atp_dot_atb / atb2, 0, 1);

    return vector_plus(pa, vector_multiply(aToB, tx));
}*/

FlowLine flowline_move(FlowLine t, Vector moveVector) {
    return new_FlowLine(
            new_Anchor(vector_plus(t.pa.point, moveVector), t.pa.radius),
            new_Anchor(vector_plus(t.pb.point, moveVector), t.pb.radius)
    );
}

/***
 *    ##     ##    ###    ########  ####    ###    ########  ##       ########  ######
 *    ##     ##   ## ##   ##     ##  ##    ## ##   ##     ## ##       ##       ##    ##
 *    ##     ##  ##   ##  ##     ##  ##   ##   ##  ##     ## ##       ##       ##
 *    ##     ## ##     ## ########   ##  ##     ## ########  ##       ######    ######
 *     ##   ##  ######### ##   ##    ##  ######### ##     ## ##       ##             ##
 *      ## ##   ##     ## ##    ##   ##  ##     ## ##     ## ##       ##       ##    ##
 *       ###    ##     ## ##     ## #### ##     ## ########  ######## ########  ######
 */

//========== COSPACE ==========
int Duration = 0;
int SuperDuration = 0;
int bGameEnd = false;
int CurAction = -1;
int CurGame = 0;
int SuperObj_Num = 0;
int SuperObj_X = 0;
int SuperObj_Y = 0;
int Teleport = 0;
int LoadedObjects = 0;
int US_Front = 0;
int US_Left = 0;
int US_Right = 0;
int CSLeft_R = 0;
int CSLeft_G = 0;
int CSLeft_B = 0;
int CSRight_R = 0;
int CSRight_G = 0;
int CSRight_B = 0;
int PX = 0;
int PY = 0;
int TM_State = 0;
int Compass = 0;
int Time = 0;
int WheelLeft = 0;
int WheelRight = 0;
int LED_1 = 0;
int MyState = 0;
int AI_SensorNum = 13;

//========== CONSTANTS ==========
#define BORDER_MARGIN 20
#define MAP_WIDTH 350
#define MAP_HEIGHT 260

#define ACTION_DEPOSIT 50
#define ACTION_DEPOSITING 51
#define ACTION_ADJUST_FOR_DEPOSIT 52
#define ACTION_COLLECT 20
#define ACTION_COLLECTING 21
#define ACTION_OBSTACLE_AVOID 30
#define ACTION_OBSTACLE_AVOIDING 31
#define ACTION_FOLLOW_ROUTE 41
#define ACTION_FOLLOW_SUPEROBJECT 61
#define ACTION_REVERSING 71
#define ACTION_NORMAL 1

int MIN_DEP_LOADED_OBJECTS = 4;
int STD_SPEED = 2;
double STD_ANGLE_TOLERANCE = 8 * M_PI / 180;
double STD_RANDOMNESS = 0.1;
int AVOIDING_BORDER_TIME = 20;
int DEPOSITING_TIME = 44;
int RANDOM_COORDINATES_PADDING = 30;
int US_DISTANCE = 3;
int ROUTE_DISTANCE_THRESHOLD = 8;
int SMALL_RADIUS = 4;
int ROBOT_WIDTH = 15;
/**
 * Collect policy = maximum number of collected objects of one color
 *  0 - all
 *  1 - space for 1 of each
 *  2 - space for 2 of each
 */
int POLICY_COLLECT = 1;
double SUPEROBJECT_VISION_DISTANCE = 200;

double BORDER_DISTANCE = 12;
double COEFF_K = 6;
double DISCRETE_FLOWPOINT_FORCE = 2.0;
int RANDOM_VECTOR_STEP_DEVIATION = 20; // degrees
double RANDOM_VECTOR_SIZE = 0.4;

//========== PROGRAM variables ==========
bool initialized = false;
int ticks = 0;

// Collecting objects
int loadedColor[] = {0, 0, 0}; // red, black, blue
int loadedSuperobject = 0;

// Superobjects
int superobjectCount = 0;
Vector superobjects[20];
Vector lastSuperobjectRegistered = {-1, -1};
bool isFollowingSuperobject = false;
FlowLine superobjectFlowLine = {};
int superobjectIndex = NONE;
int lastDecidedForSuperobject = 10000000;

//========== STATE variables ==========
int collectingTime = 0;
bool mustRemainCollecting = true;
int currentArea = 0;
int currentCheckpoint = 0;
int avoidingObstacleTime = 0;
int avoidingBorderTime = 0;
Vector *avoidingBorderPos;
int depositingTime = 0;
int reversingTime = 0;
int currentRoute = NONE;
int currentRoutePoint = NONE;

// moving and position
Vector currentPosition;
Direction currentDirection;
Vector lastPosition;
Direction lastDirection;
Vector estimatedPosition;
Direction estimatedDirection;
int motorLeft;
int motorRight;
Direction lastRandomDirection;

// Flows
int bottomEnvIndex;
int depositEnvIndex = NONE;

//========== TEMPORARY variables ==========
int lastState = 0;
int debug1 = -777;
int debug2 = -777;

//========== REAL PART ==========
int kanyar = 0;
int goingtodeposit = 0;
int start = 0;
int foundDeposit = 0;
int justdeposited = 0;

/***
 *    ########  ########  ######## ########  ######## ######## #### ##    ## ######## ########
 *    ##     ## ##     ## ##       ##     ## ##       ##        ##  ###   ## ##       ##     ##
 *    ##     ## ##     ## ##       ##     ## ##       ##        ##  ####  ## ##       ##     ##
 *    ########  ########  ######   ##     ## ######   ######    ##  ## ## ## ######   ##     ##
 *    ##        ##   ##   ##       ##     ## ##       ##        ##  ##  #### ##       ##     ##
 *    ##        ##    ##  ##       ##     ## ##       ##        ##  ##   ### ##       ##     ##
 *    ##        ##     ## ######## ########  ######## ##       #### ##    ## ######## ########
 */

bool _was_init = false;

#define AREAS_COUNT 10
Area AREAS[AREAS_COUNT];
int _index_area = 0;

void _area(int x1, int y1, int x2, int y2) {
    AREAS[_index_area++] = new_area(x1, y1, x2, y2);
}

// ========== ENVIRONMENTS ==========
#define ENVIRONMENT_COUNT 3
Environment ENVIRONMENTS[ENVIRONMENT_COUNT];

int environment_count = 0;

Environment *getEnv(int index) {
    return &ENVIRONMENTS[index];
}

int getCurrentEnvIndex() {
    if (depositEnvIndex != NONE) {
        return 0;
    }
    return bottomEnvIndex;
}

Environment *getCurrentEnv() {
    return getEnv(getCurrentEnvIndex());
}

void setBottomEnv(int index) {
    bottomEnvIndex = index;
}

void _environment(double randomnessSize) { // _environment must be defined before _anchor, _flowPoint
    Environment *e = &ENVIRONMENTS[environment_count++];
    e->randomnessSize = randomnessSize;
}

void _anchor(int x, int y, int radius) {
    Environment *e = &ENVIRONMENTS[environment_count - 1];
    e->anchors[e->anchorCount++] = new_Anchor(new_vector(x, y), radius);
}

void _flowPoint(int x, int y, int radius, int direction) {
    Environment *e = &ENVIRONMENTS[environment_count - 1];
    e->flowPoints[e->flowPointCount++] = new_FlowPoint(new_vector(x, y), radius, direction_fromDegrees(direction));
}

void _init_flowlines() {
    for (int ei = 0; ei < ENVIRONMENT_COUNT; ei++) {
        Environment *e = &ENVIRONMENTS[ei];
        for (int i = 0; i < e->anchorCount; i++) {
            Anchor aa = e->anchors[i];
            Anchor ab = e->anchors[(i + 1) % e->anchorCount];
            e->flowLines[i] = new_FlowLine(aa, ab);
        }
    }
}

void _environment_route(bool withEnd) {
    Environment *e = &ENVIRONMENTS[environment_count - 1];
    e->flowRoutes[e->routeCount++].withEnd = withEnd;
}

void _environment_route_point(int x, int y, int radius) {
    Environment *e = &ENVIRONMENTS[environment_count - 1];
    FlowRoute *r = &(e->flowRoutes[e->routeCount - 1]);
    r->points[r->count++] = new_Anchor(new_vector(x, y), radius);
}


// ========== ROUTES ==========

#define ROUTES_COUNT 1 // routes: put number of routes here
Route ROUTES[ROUTES_COUNT];

Route *getCurrentRoute() {
    if (currentRoute == NONE) return NULL;
    return &ROUTES[currentRoute];
}

void _routePoint(int route, int x, int y) {
    Route *r = &ROUTES[route];
    r->points[r->count++] = new_vector(x, y);
}

// ============= WALLS =============
#define WALLS_COUNT 100
Wall WALLS[WALLS_COUNT];

int wall_count = 0;

void _wall(int ax, int ay, int bx, int by) {
    WALLS[wall_count++] = new_Wall(new_vector(ax, ay), new_vector(bx, by));
};

// ========== SUPEROBJECTRULES ==========

SuperobjectRule SUPEROBJECT_RULES[20];
int superobjectRulesCount = 0;

void _rule(int areaXa, int areaYa, int areaXb, int areaYb, int entryXa, int entryYa, int entryXb, int entryYb) {
    SuperobjectRule *r = &SUPEROBJECT_RULES[superobjectRulesCount++];
    r->area = new_area(areaXa, areaYa, areaXb, areaYb);
    r->entry = new_area(entryXa, entryYa, entryXb, entryYb);
    r->route.count = 0;
}

void _rule_route_point(int x, int y) { // _rule must be defined first
    // add the route point to the last SuperobjectRule
    SuperobjectRule *r = &SUPEROBJECT_RULES[superobjectRulesCount - 1];
    r->route.points[r->route.count++] = new_vector(x, y);
}

// ========== INITIALIZATION ==========

void _init_values() {

    // ===== GLOBAL =========

    //Wall: left stone
    _wall(0, 94, 40, 94);
    _wall(40, 94, 39, 104);
    _wall(39, 104, 0, 104);
    _wall(0, 104, 0, 94);

    //Wall: left down stone strap trap
    _wall(40, 0, 50, 0);
    _wall(50, 0, 50, 40);
    _wall(50, 40, 70, 40);
    _wall(70, 40, 70, 60);
    _wall(70, 60, 100, 60);
    _wall(100, 60, 100, 90);
    _wall(100, 90, 70, 90);
    _wall(70, 90, 70, 70);
    _wall(70, 70, 40, 70);
    _wall(40, 70, 40, 0);

    //Wall: down doublestone stone
    _wall(140, 60, 220, 60);
    _wall(220, 60, 220, 30);
    _wall(220, 30, 230, 30);
    _wall(230, 30, 230, 70);
    _wall(230, 70, 140, 70);
    _wall(140, 70, 140, 60);

    //Wall: down stone
    _wall(310, 0, 320, 0);
    _wall(320, 0, 320, 40);
    _wall(320, 40, 310, 40);
    _wall(310, 40, 310, 0);

    //Wall: down swamp cross
    _wall(231, 30, 250, 0);

    //Wall: tower stone trap trap stones around up right deposite)
    _wall(170, 200, 175, 187);
    _wall(175, 187, 195, 187);
    _wall(195, 187, 195, 195);
    _wall(195, 195, 258, 195);
    _wall(258, 195, 282, 158);
    _wall(282, 158, 291, 165);
    _wall(291, 165, 275, 188);
    _wall(275, 188, 297, 206);
    _wall(297, 206, 314, 181);
    _wall(314, 181, 324, 187);
    _wall(324, 187, 305, 216);
    _wall(305, 216, 295, 244);
    _wall(295, 244, 265, 244);
    _wall(265, 244, 265, 228);
    _wall(265, 228, 234, 227);
    _wall(234, 227, 234, 207);
    _wall(234, 207, 195, 207);
    _wall(195, 207, 190, 209);
    _wall(190, 209, 177, 213);
    _wall(177, 213, 170, 200);

    //Wall: up swamp cross
    _wall(127, 181, 170, 200);

    //Wall: right trap

    _wall(330, 70, 360, 70);
    _wall(360, 70, 360, 100);
    _wall(360, 100, 330, 100);
    _wall(330, 100, 330, 70);

    //Wall: up stone
    _wall(110, 230, 120, 230);
    _wall(120, 230, 121, 270);
    _wall(121, 270, 110, 270);
    _wall(110, 270, 110, 230);

    //Quick areas
    _area(0, 175, 140, 110);
    _area(240, 143, 324, 45);
    _area(320, 175, 357, 99);

    _area(112, 114, 137, 73);

    // ===== ENVIRONMENT: deposit
    _environment(0.0);

    _environment_route(true);
    _environment_route_point(61, 220, 50);
    _environment_route_point(66, 108, 10);
    _environment_route_point(52, 86, 10);
    _environment_route_point(18, 71, 5);
    _environment_route_point(12, 21, 5);

    _environment_route(false);
    _environment_route_point(213, 33, 20);
    _environment_route_point(117, 35, 20);
    _environment_route_point(119, 108, 10);
    _environment_route_point(66, 108, 10);
    _environment_route_point(51, 87, 10);

    _environment_route(true);
    _environment_route_point(140, 221, 4);
    _environment_route_point(163, 249, 15);
    _environment_route_point(281, 262, 4);
    _environment_route_point(340, 248, 4);
    _environment_route_point(344, 180, 6);
    _environment_route_point(327, 163, 10);
    _environment_route_point(312, 161, 4);
    _environment_route_point(295, 184, 4);

    _environment_route(true);
    _environment_route_point(149, 155, 30);
    _environment_route_point(241, 120, 30);
    _environment_route_point(299, 144, 10);
    _environment_route_point(312, 161, 4);
    _environment_route_point(295, 184, 4);

    _environment_route(true);
    _environment_route_point(253, 61, 20);
    _environment_route_point(339, 55, 10);
    _environment_route_point(342, 19, 5);

    _flowPoint(110, 234, 15, -135);
    _flowPoint(111, 147, 40, -135);
    _flowPoint(25, 120, 40, 15);
    _flowPoint(75, 40, 30, 0);
    _flowPoint(65, 20, 30, 0);
    _flowPoint(97, 60, 20, -10);
    _flowPoint(70, 91, 12, 175);
    _flowPoint(65, 77, 25, 170);
    _flowPoint(78, 60, 20, -20);
    _flowPoint(92, 258, 30, -135);
    _flowPoint(121, 190, 35, 160);
    _flowPoint(125, 198, 20, 135);
    _flowPoint(144, 61, 30, -145);
    _flowPoint(179, 59, 30, -135);
    _flowPoint(217, 59, 30, -135);
    _flowPoint(177, 31, 50, 180);
    _flowPoint(226, 12, 30, 180);

    _flowPoint(311, 35, 20, 120);
    _flowPoint(316, 90, 18, 80);
    _flowPoint(327, 108, 25, 112);
    _flowPoint(174, 216, 25, 75);
    _flowPoint(222, 222, 30, 85);
    _flowPoint(255, 235, 43, 105);
    _flowPoint(289, 240, 24, 55);
    _flowPoint(314, 229, 30, 0);
    _flowPoint(326, 189, 20, -50);
    _flowPoint(231, 176, 40, -55);
    _flowPoint(267, 161, 30, -80);
    _flowPoint(323, 208, 28, 0);
    _flowPoint(255, 187, 20, -105);
    _flowPoint(315, 72, 25, -75);
    _flowPoint(270, 216, 30, 135);
    _flowPoint(267, 247, 25,65);
    _flowPoint(270, 247, 25,50);
    _flowPoint(305, 244, 35, 45);
    _flowPoint(234, 235, 40, 95);
    _flowPoint(113, 175, 20, -135);
    _flowPoint(99, 215, 30, -135);
    _flowPoint(99, 91, 13, 45);

    // ===== ENVIRONMENT: normal 1
    _environment(STD_RANDOMNESS);

    _environment_route(true);
    _environment_route_point(14, 252, 20);
    _environment_route_point(75, 220, 25);
    _environment_route_point(100, 216, 6);
    _environment_route_point(140, 216, 15);
    _environment_route_point(176, 257, 21);
    _environment_route_point(260, 260, 10);
    _environment_route_point(322, 257, 10);
    _environment_route_point(342, 211, 12);
    _environment_route_point(343, 172, 6);
    _environment_route_point(271, 127, 25);
    _environment_route_point(147, 127, 40);
    _environment_route_point(120, 95, 8);
    _environment_route_point(117, 44, 15);
    _environment_route_point(139, 17, 15);
    _environment_route_point(212, 14, 19);

    _environment_route(false);
    _environment_route_point(12, 22, 6);
    _environment_route_point(18, 73, 4);
    _environment_route_point(53, 85, 5);
    _environment_route_point(67, 108, 12);
    _environment_route_point(108, 106, 18);

    _environment_route(false);
    _environment_route_point(346, 18, 8);
    _environment_route_point(339, 52, 8);
    _environment_route_point(278, 72, 8);
    _environment_route_point(246, 122, 14);

    _flowPoint(115, 195, 22, 135);
    _flowPoint(116, 176, 20, -115);
    _flowPoint(173, 178, 27, -70);
    _flowPoint(25, 110, 15, 40);
    _flowPoint(46, 65, 18, 135);
    _flowPoint(75, 84, 14, 145);
    _flowPoint(90, 83, 15, 33);
    _flowPoint(68, 31, 23, -30);
    _flowPoint(146, 74, 12, 150);
    _flowPoint(252, 42, 60, 58);
    _flowPoint(285, 39, 63, 80);
    _flowPoint(233, 73, 20, 60);
    _flowPoint(336, 98, 14, 135);
    _flowPoint(337, 77, 14, -135);
    _flowPoint(88, 60, 17, -60);
    _flowPoint(233, 220, 22, 105);
    _flowPoint(265, 239, 24, 110);
    _flowPoint(264, 218, 30, 123);
    _flowPoint(83, 182, 40, 85);
    _flowPoint(26, 185, 35, 70);
    _flowPoint(318, 191, 18, 10);
    _flowPoint(101, 244, 28, -115);
    _flowPoint(143, 176, 19, -98);
    _flowPoint(153, 197, 33, 115);
    _flowPoint(35, 95, 12, -45);
    _flowPoint(332, 20, 20, 78);
    _flowPoint(283, 167, 18, -85);
    _flowPoint(301, 180, 24, -50);
    _flowPoint(293, 229, 20, 20);
    _flowPoint(203, 46, 20, -127);
    _flowPoint(150, 49, 18, -110);
    _flowPoint(302, 113, 35, 160);
    _flowPoint(285, 245, 16, 50);
    _flowPoint(248, 52, 35, 40);

    // ===== ENVIRONMENT: normal 2
    _environment(STD_RANDOMNESS);

    //flow from down left deposit to upper part
    _environment_route(false);
    _environment_route_point(12, 22, 10);
    _environment_route_point(18, 72, 8);
    _environment_route_point(53, 85, 7);
    _environment_route_point(58, 114, 12);
    _environment_route_point(30, 191, 18);


    //flow from down bonus to up
    _environment_route(true);
    _environment_route_point(212, 14, 20);
    _environment_route_point(194, 35, 15);
    _environment_route_point(129, 46, 10);
    _environment_route_point(120, 112, 10);
    _environment_route_point(30, 191, 25);
    _environment_route_point(9, 259, 20);

    // /flow from very right via uppper part to up left
    _environment_route(true);
    _environment_route_point(343, 172, 6);
    _environment_route_point(339, 207, 13);
    _environment_route_point(307, 260, 10);
    _environment_route_point(260, 260, 6);
    _environment_route_point(181, 252, 25);
    _environment_route_point(140, 216, 15);
    _environment_route_point(100, 216, 6);
    _environment_route_point(75, 220, 25);
    _environment_route_point(9, 259, 20);

    //flow from down right deposit to up left
    _environment_route(false);
    _environment_route_point(340, 22, 10);
    _environment_route_point(337, 53, 10);
    _environment_route_point(256, 107, 30);
    _environment_route_point(121, 130, 40);

    _flowPoint(297, 181, 30, -53);
    _flowPoint(182, 178, 20, -90);
    _flowPoint(84, 50, 18, -18);
    _flowPoint(62, 30, 20, -10);
    _flowPoint(97, 63, 18, -18);
    _flowPoint(230, 12, 20, 180);
    _flowPoint(260, 20, 20, 90);
    _flowPoint(248, 51, 40, 65);
    _flowPoint(258, 49, 35, 75);
    _flowPoint(237, 74, 20, 80);
    _flowPoint(125, 194, 22, 160);
    _flowPoint(152, 179, 20, -80);
    _flowPoint(166, 179, 20, -65);
    _flowPoint(132, 176, 10, -80);
    _flowPoint(129, 252, 30, -65);
    _flowPoint(128, 235, 20, -75);
    _flowPoint(331, 70, 18, -160);
    _flowPoint(293, 238, 18, 82);
    _flowPoint(155, 60, 19, -137);
    _flowPoint(45, 62, 16, 125);
    _flowPoint(72, 84, 14, 120);
    _flowPoint(28, 95, 18, -40);
    _flowPoint(284, 167, 18, -65);
    _flowPoint(248, 183, 30, -115);
    _flowPoint(207, 180, 25, -110);
    _flowPoint(245, 217, 18, 100);
    _flowPoint(321, 185, 12, -10);
    _flowPoint(318, 152, 28, 30);
    _flowPoint(116, 170, 10, -155);
    _flowPoint(338, 126, 30, 75);
    _flowPoint(196, 59, 24, -145);
    _flowPoint(96, 87, 18, 35);

}

/***
 *    ######## ##     ## ##    ##  ######  ######## ####  #######  ##    ##  ######
 *    ##       ##     ## ###   ## ##    ##    ##     ##  ##     ## ###   ## ##    ##
 *    ##       ##     ## ####  ## ##          ##     ##  ##     ## ####  ## ##
 *    ######   ##     ## ## ## ## ##          ##     ##  ##     ## ## ## ##  ######
 *    ##       ##     ## ##  #### ##          ##     ##  ##     ## ##  ####       ##
 *    ##       ##     ## ##   ### ##    ##    ##     ##  ##     ## ##   ### ##    ##
 *    ##        #######  ##    ##  ######     ##    ####  #######  ##    ##  ######
 */

Vector getEstimatedPosition();

// ========== COMPUTATION ==========

typedef struct {
    uint32_t s1, s2, s3;
} rnd_state;

rnd_state *new_rnd_state() {
    rnd_state *o = malloc(sizeof(rnd_state));
    o->s1 = 51635231;
    o->s2 = 8486513;
    o->s3 = 8521688;
    return o;
}

rnd_state rnd_state_o;

static uint32_t __random32() {
#define TAUSWORTHE(s, a, b, c, d) ((((s)&(c))<<(d)) ^ ((((s) <<(a)) ^ (s))>>(b)))
    rnd_state *state = &rnd_state_o;
    state->s1 = TAUSWORTHE(state->s1, 13, 19, 4294967294UL, 12);
    state->s2 = TAUSWORTHE(state->s2, 2, 25, 4294967288UL, 4);
    state->s3 = TAUSWORTHE(state->s3, 3, 11, 4294967280UL, 17);
    return (state->s1 ^ state->s2 ^ state->s3);
}

int randn(int n) {
    return __random32() % n;
}

int angleDiff(int a, int b) {
    int d1 = b - a;
    if (d1 > 180) d1 -= 360;
    if (d1 < -180) d1 += 360;
    return d1;
}

int angleTo(Vector *p) {
    int a = (int) (atan2(p->x - PX, p->y - PY) * 180 / 3.141592658);
    if (a < 0) a += 360;
    return a;
}

double toRange(double value, double mmin, double mmax) {
    return min(max(mmin, value), mmax);
}

int toRangeInt(int value, int mmin, int mmax) {
    return min(max(mmin, value), mmax);
}

double abs_double(double value) {
    return value >= 0 ? value : -value;
}

bool isInArea(Vector point, Area area) {
    return area.xa <= point.x && area.xb >= point.x && area.ya <= point.y && area.yb >= point.y;
}

bool isInRect(Vector point, int xa, int ya, int xb, int yb) {
    return isInArea(point, new_area(xa, ya, xb, yb));
}

// =========== COLORS ============

/*bool isYellowRight() { return CSRight_R > 200 && CSRight_G > 200 && CSRight_B < 50; }
bool isYellowLeft() { return CSLeft_R > 200 && CSLeft_G > 200 && CSLeft_B < 50; }
bool isYellow() { return isYellowLeft() || isYellowRight(); }
bool isRedRight() { return CSRight_R > 200 && CSRight_G < 50 && CSRight_B < 50; }
bool isRedLeft() { return CSLeft_R > 200 && CSLeft_G < 50 && CSLeft_B < 50; }
bool isRed() { return isRedRight() || isRedLeft(); }
bool isGreenRight() { return CSRight_R < 50 && CSLeft_G > 200 && CSRight_B < 50; }
bool isGreenLeft() { return CSLeft_R < 50 && CSLeft_G > 200 && CSLeft_B < 50; }
bool isGreen() { return isGreenRight() || isGreenLeft(); }
bool isBlackRight() { return CSRight_R < 50 && CSRight_G < 50 && CSRight_B < 50; }
bool isBlackLeft() { return CSLeft_R < 50 && CSLeft_G < 50 && CSLeft_B < 50; }
bool isBlack() { return isBlackRight() || isBlackLeft(); }
bool isBlueRight() { return ((CSRight_R < 5) && (CSRight_G > 140 && CSRight_G < 180) && (CSRight_B > 250)); }
bool isBlueLeft() { return ((CSLeft_R < 5) && (CSLeft_G > 140 && CSLeft_G < 180) && (CSLeft_B > 250)); }
bool isBlue() { return isBlueLeft() && isBlueRight(); }
bool isOrangeRight() {
    return ((CSRight_R > 200 && CSRight_R < 240) && (CSRight_G > 80 && CSRight_G < 110) && (CSRight_B < 5));
}
bool isOrangeLeft() {
    return ((CSLeft_R > 200 && CSLeft_R < 240) && (CSLeft_G > 80 && CSLeft_G < 110) && (CSLeft_B < 5));
}
bool isOrange() { return isOrangeRight() || isOrangeLeft(); }*/

bool isOrangeRight() {
    return (((CSRight_R > 204 - 10) && (CSRight_R < 235 + 10)) && ((CSRight_G > 163 - 10) && (CSRight_G < 186 + 10)) &&
            ((CSRight_B > 0 - 10) && (CSRight_B < 0 + 10)));
}

bool isOrangeLeft() {
    return (((CSLeft_R > 204 - 10) && (CSLeft_R < 235 + 10)) && ((CSLeft_G > 163 - 10) && (CSLeft_G < 186 + 10)) &&
            ((CSLeft_B > 0 - 10) && (CSLeft_B < 0 + 10)));
}

bool isOrange() { return (isOrangeLeft() || isOrangeRight()); }

bool isGreyRight() {
    return (((CSRight_R > 133 - 10) && (CSRight_R < 153 + 10)) && ((CSRight_G > 141 - 10) && (CSRight_G < 161 + 10)) &&
            ((CSRight_B > 187 - 10) && (CSRight_B < 207 + 10)));
}

bool isGreyLeft() {
    return (((CSLeft_R > 133 - 10) && (CSLeft_R < 153 + 10)) && ((CSLeft_G > 141 - 10) && (CSLeft_G < 161 + 10)) &&
            ((CSLeft_B > 187 - 10) && (CSLeft_B < 207 + 10)));
}

bool isGrey() { return (isGreyLeft() || isGreyRight()); }

bool isYellowRight() {
    return (((CSRight_R > 204 - 10) && (CSRight_R < 235 + 10)) && ((CSRight_G > 217 - 10) && (CSRight_G < 248 + 10)) &&
            ((CSRight_B > 0 - 10) && (CSRight_B < 0 + 10)));
}

bool isYellowLeft() {
    return (((CSLeft_R > 204 - 10) && (CSLeft_R < 235 + 10)) && ((CSLeft_G > 217 - 10) && (CSLeft_G < 248 + 10)) &&
            ((CSLeft_B > 0 - 10) && (CSLeft_B < 0 + 10)));
}

bool isYellow() { return (isYellowLeft() || isYellowRight()); }

bool isBlackRight() {
    return (((CSRight_R > 29 - 10) && (CSRight_R < 39 + 10)) && ((CSRight_G > 29 - 10) && (CSRight_G < 39 + 10)) &&
            ((CSRight_B > 29 - 10) && (CSRight_B < 39 + 10)));
}

bool isBlackLeft() {
    return (((CSLeft_R > 29 - 10) && (CSLeft_R < 39 + 10)) && ((CSLeft_G > 29 - 10) && (CSLeft_G < 39 + 10)) &&
            ((CSLeft_B > 29 - 10) && (CSLeft_B < 39 + 10)));
}

bool isBlack() { return (isBlackLeft() || isBlackRight()); }

bool isDarkBlueRight() {
    return (((CSRight_R > 0 - 10) && (CSRight_R < 0 + 10)) && ((CSRight_G > 150 - 10) && (CSRight_G < 171 + 10)) &&
            ((CSRight_B > 255 - 10) && (CSRight_B < 255 + 10)));
}

bool isDarkBlueLeft() {
    return (((CSLeft_R > 0 - 10) && (CSLeft_R < 0 + 10)) && ((CSLeft_G > 150 - 10) && (CSLeft_G < 171 + 10)) &&
            ((CSLeft_B > 255 - 10) && (CSLeft_B < 255 + 10)));
}

bool isDarkBlue() { return (isDarkBlueLeft() || isDarkBlueRight()); }

bool isRedRight() {
    return (((CSRight_R > 232 - 10) && (CSRight_R < 255 + 10)) && ((CSRight_G > 29 - 10) && (CSRight_G < 39 + 10)) &&
            ((CSRight_B > 29 - 10) && (CSRight_B < 39 + 10)));
}

bool isRedLeft() {
    return (((CSLeft_R > 232 - 10) && (CSLeft_R < 255 + 10)) && ((CSLeft_G > 29 - 10) && (CSLeft_G < 39 + 10)) &&
            ((CSLeft_B > 29 - 10) && (CSLeft_B < 39 + 10)));
}

bool isRed() { return (isRedLeft() || isRedRight()); }

bool isLightGrayRight() {
    return (((CSRight_R > 180 - 10) && (CSRight_R < 206 + 10)) && ((CSRight_G > 191 - 10) && (CSRight_G < 217 + 10)) &&
            ((CSRight_B > 251 - 10) && (CSRight_B < 255 + 10)));
}

bool isLightGrayLeft() {
    return (((CSLeft_R > 180 - 10) && (CSLeft_R < 206 + 10)) && ((CSLeft_G > 191 - 10) && (CSLeft_G < 217 + 10)) &&
            ((CSLeft_B > 251 - 10) && (CSLeft_B < 255 + 10)));
}

bool isLightGray() { return (isLightGrayLeft() || isLightGrayRight()); }

bool isBlueRight() {
    return (((CSRight_R > 29 - 10) && (CSRight_R < 39 + 10)) && ((CSRight_G > 249 - 10) && (CSRight_G < 255 + 10)) &&
            ((CSRight_B > 255 - 10) && (CSRight_B < 255 + 10)));
}

bool isBlueLeft() {
    return (((CSLeft_R > 29 - 10) && (CSLeft_R < 39 + 10)) && ((CSLeft_G > 249 - 10) && (CSLeft_G < 255 + 10)) &&
            ((CSLeft_B > 255 - 10) && (CSLeft_B < 255 + 10)));
}

bool isBlue() { return (isBlueLeft() || isBlueRight()); }

bool isVioletRight() {
    return (((CSRight_R > 232 - 10) && (CSRight_R < 250 + 10)) && ((CSRight_G > 30 - 10) && (CSRight_G < 41 + 10)) &&
            ((CSRight_B > 255 - 10) && (CSRight_B < 255 + 10)));
}

bool isVioletLeft() {
    return (((CSLeft_R > 232 - 10) && (CSLeft_R < 250 + 10)) && ((CSLeft_G > 30 - 10) && (CSLeft_G < 41 + 10)) &&
            ((CSLeft_B > 255 - 10) && (CSLeft_B < 255 + 10)));
}

bool isViolet() { return (isVioletLeft() || isVioletRight()); }

// =========== AREAS ==========

Vector randomCoordinates(int arean) {
    Area a = AREAS[arean];
    int x = randn(a.xb - a.xa - 2 * RANDOM_COORDINATES_PADDING) + a.xa + RANDOM_COORDINATES_PADDING;
    int y = randn(a.yb - a.ya - 2 * RANDOM_COORDINATES_PADDING) + a.ya + RANDOM_COORDINATES_PADDING;
    return new_vector(x, y);
}

// =========== CHECKS ==========

bool canCollect() {
    return (isRed() || isBlack() || isBlue() || isViolet()) && LoadedObjects < 6;
}

bool shouldCollect() {
    int index;
    if (isRed()) {
        index = 0;
    } else if (isBlack()) {
        index = 1;
    } else if (isBlue()) {
        index = 2;
    } else {
        // always collect superobject
        return true;
    }
    int futureFreeSpace = 6 - LoadedObjects - 1;
    int needToCollect = 3 * POLICY_COLLECT
                        - min(loadedColor[0], POLICY_COLLECT)
                        - min(loadedColor[1], POLICY_COLLECT)
                        - min(loadedColor[2], POLICY_COLLECT);
    int afterNeedToCollect = needToCollect;
    if (loadedColor[index] < POLICY_COLLECT)
        afterNeedToCollect -= 1;
    return ((futureFreeSpace >= afterNeedToCollect) || (6 - LoadedObjects < needToCollect))
           && (!isFollowingSuperobject || LoadedObjects <= 4);
}

void registerCollect() {
    LoadedObjects++;
    if (isRed()) {
        loadedColor[0]++;
    } else if (isBlack()) {
        loadedColor[1]++;
    } else if (isBlue()) {
        loadedColor[2]++;
    } else if (isViolet()) {
        loadedSuperobject++;
    }
}

void registerDeposit() {
    LoadedObjects = 0;
    loadedColor[0] = 0;
    loadedColor[1] = 0;
    loadedColor[2] = 0;
    loadedSuperobject = 0;
}

bool seesObstacleLeft() {
    return US_Left < US_DISTANCE;
}

bool seesObstacleFront() {
    return US_Front < US_DISTANCE;
}

bool seesObstacleRight() {
    return US_Right < US_DISTANCE;
}

bool shouldAvoidObstacle() {
    return seesObstacleLeft() || seesObstacleFront() || seesObstacleRight() || (LoadedObjects > 1 && isYellow());
}

bool shouldAvoidBorder(int arean) {
    Area a = AREAS[arean];
    return PX < a.xa || PY < a.ya || PX > a.xb || PY > a.yb;
}

bool shouldFollowNextDeposit() {
    return LoadedObjects >= MIN_DEP_LOADED_OBJECTS;
}

bool shouldDeposit() {
    return LoadedObjects > 0;
}

bool wantsDeposit() {
    return LoadedObjects >= 6
           || (LoadedObjects >= 4
               && loadedColor[0] >= POLICY_COLLECT && loadedColor[1] >= POLICY_COLLECT &&
               loadedColor[2] >= POLICY_COLLECT
               && loadedSuperobject >= 1);
}

bool canDeposit() {
    return isOrangeLeft() && isOrangeRight();
}

bool isInFastArea() {
    Vector pos = getEstimatedPosition();
    for (int i = 0; i < _index_area; i++) {
        if (isInArea(pos, AREAS[i])) {
            return true;
        }
    }
    return false;
}

// ========== POSITION ===========

void saveCurrentPositionInfo() {
    currentPosition = new_vector(PX, PY);
    currentDirection = direction_fromDegrees(Compass + 90);
}

Vector getCurrentPosition() {
    return currentPosition;
}

Direction getCurrentDirection() {
    return currentDirection;
}

Vector getEstimatedPosition() {
    return estimatedPosition;
}

Direction getEstimatedDirection() {
    return estimatedDirection;
}

bool isPositionKnown() {
    return currentPosition.x != 0 || currentPosition.y != 0;
}

void observePosition() {
    saveCurrentPositionInfo();
    if (isPositionKnown()) {
        lastPosition = estimatedPosition = getCurrentPosition();
        lastDirection = estimatedDirection = getCurrentDirection();
    } else {
        double forward = (WheelLeft + WheelRight) / 2;
        estimatedPosition = vector_plus(estimatedPosition, vector_radial(estimatedDirection, 0.6 * forward));
        double rotation = (WheelRight - WheelLeft) / 2;
        estimatedDirection = direction_plus(estimatedDirection, direction_fromDegrees(rotation * 4.48));
    }
}

int estPosX() {
    return (int) getEstimatedPosition().x;
}

int estPosY() {
    return (int) getEstimatedPosition().y;
}

int estDir() {
    return (int) direction_degrees(getEstimatedDirection());
}

// ========== SUPEROBJECTS ==========

void registerSuperobject() {
    if (SuperObj_Num == 1 && (
            SuperObj_X != lastSuperobjectRegistered.x && SuperObj_Y != lastSuperobjectRegistered.y
    )) {
        Vector superobject = new_vector(SuperObj_X + 10, SuperObj_Y);
        lastSuperobjectRegistered = superobject;
        superobjects[superobjectCount++] = superobject;
    }
}

void unregisterSuperobject(int index) {
    // if it's not the last, move the last to the 'index' position
    if (index != superobjectCount - 1) {
        superobjects[index] = superobjects[superobjectCount - 1];
    }
    superobjectCount--;
}

bool findIntersection(double p0_x, double p0_y, double p1_x, double p1_y,
                      double p2_x, double p2_y, double p3_x, double p3_y) {
    double s1_x, s1_y, s2_x, s2_y;
    s1_x = p1_x - p0_x;
    s1_y = p1_y - p0_y;
    s2_x = p3_x - p2_x;
    s2_y = p3_y - p2_y;

    double s, t;
    s = (-s1_y * (p0_x - p2_x) + s1_x * (p0_y - p2_y)) / (-s2_x * s1_y + s1_x * s2_y);
    t = (s2_x * (p0_y - p2_y) - s2_y * (p0_x - p2_x)) / (-s2_x * s1_y + s1_x * s2_y);

    if (s >= 0 && s <= 1 && t >= 0 && t <= 1) {
        return true;
    }
    return 0; // No collision
}

bool isNotObstructed(Vector a, Vector b) {
    Direction direction = vector_directionTo(a, b);
    Vector moveA = vector_radial(direction_plus_double(direction, M_PI / 2), ROBOT_WIDTH / 2);
    Vector moveB = vector_radial(direction_plus_double(direction, -M_PI / 2), ROBOT_WIDTH / 2);
    Vector aMovedA = vector_plus(a, moveA);
    Vector aMovedB = vector_plus(a, moveB);
    Vector bMovedA = vector_plus(b, moveA);
    Vector bMovedB = vector_plus(b, moveB);
    for (int i = 0; i < wall_count; i++) {
        Wall wall = WALLS[i];
        if (findIntersection(
                wall.a.x, wall.a.y, wall.b.x, wall.b.y,
                aMovedA.x, aMovedA.y, bMovedA.x, bMovedA.y
        ) || findIntersection(
                wall.a.x, wall.a.y, wall.b.x, wall.b.y,
                aMovedB.x, aMovedB.y, bMovedB.x, bMovedB.y
        ))
            return false;
    }
    return true;
}

void generateSuperobjectRoute() {
    Vector position = getEstimatedPosition();

    int bestSuperobjectIndex = NONE;
    double bestDistance = 0;

    if (LoadedObjects <= 5) {
        for (int index = 0; index < superobjectCount; index++) {
            Vector superobject = superobjects[index];

            if (vector_distanceTo(position, superobject) <= SUPEROBJECT_VISION_DISTANCE
                && isNotObstructed(position, superobject)) {
                double distance = vector_distanceTo(position, superobject);
                if (bestSuperobjectIndex == NONE || distance < bestDistance) {
                    bestSuperobjectIndex = index;
                    bestDistance = distance;
                }
            }
        }
    }
    if (bestSuperobjectIndex != NONE) {
        if (bestSuperobjectIndex != superobjectIndex) {
            lastDecidedForSuperobject = Time;
        }
        isFollowingSuperobject = true;
        superobjectFlowLine = new_FlowLine(
                new_Anchor(position, SMALL_RADIUS),
                new_Anchor(superobjects[bestSuperobjectIndex], SMALL_RADIUS)
        );
        superobjectIndex = bestSuperobjectIndex;
    } else {
        isFollowingSuperobject = false;
        superobjectIndex = NONE;
    }
};

void stopFollowingSuperobject() {
    unregisterSuperobject(superobjectIndex);
    isFollowingSuperobject = false;
    superobjectIndex = NONE;
}

/***
 *       ###     ######  ######## ####  #######  ##    ##  ######
 *      ## ##   ##    ##    ##     ##  ##     ## ###   ## ##    ##
 *     ##   ##  ##          ##     ##  ##     ## ####  ## ##
 *    ##     ## ##          ##     ##  ##     ## ## ## ##  ######
 *    ######### ##          ##     ##  ##     ## ##  ####       ##
 *    ##     ## ##    ##    ##     ##  ##     ## ##   ### ##    ##
 *    ##     ##  ######     ##    ####  #######  ##    ##  ######
 */

void move(int left, int right) {
    WheelLeft = toRangeInt(left, -5, 5);
    WheelRight = toRangeInt(right, -5, 5);
}

void forward(int speed) {
    move(speed, speed);
}

void stop() {
    forward(0);
}

void turn(int speed, bool toLeft) {
    if (toLeft) {
        move(-speed, speed);
    } else {
        move(speed, -speed);
    }
}

void steer(int motorA, int motorB, bool toLeft) {
    if (toLeft) {
        move(motorB, motorA);
    } else {
        move(motorA, motorB);
    }
}

double getSteerAngle(double relativeAngle) {
    if (relativeAngle >= M_PI) relativeAngle -= 2 * M_PI;
    return relativeAngle;
}

double getSteerAngleTo(Vector targetPoint) {
    Direction targetDirection = vector_directionTo(getEstimatedPosition(), targetPoint);
    return getSteerAngle(direction_differenceTo(getEstimatedDirection(), targetDirection));
}

double getAngleTolerance() {
    if (isPositionKnown()) {
        return STD_ANGLE_TOLERANCE;
    }
    return 2 * STD_ANGLE_TOLERANCE;
}

void steerWithAngle(double steerAngle) {
    int speed;
    if (isInFastArea() && !isFollowingSuperobject) {
        speed = min(STD_SPEED * 2, 5);
    } else {
        speed = STD_SPEED;
    }
    if (abs_double(steerAngle) < getAngleTolerance()) {
        if (isGrey()) {
            speed = 5;
        }
        forward(speed);
    } else {
        double k = toDegrees(abs_double(steerAngle)) / 40;
        double motorA = speed + k;
        double motorB = speed - k;
        steer((int) ceil(motorA), (int) ceil(motorB), steerAngle >= 0);
    }
}

void steerTo(Vector point) {
    double angle = getSteerAngleTo(point);
    steerWithAngle(angle);
}

void turnTo(Vector p) {
    double steerAngle = getSteerAngleTo(p);
    int steerSpeed;
    if (isPositionKnown()) {
        if (steerAngle < 40) {
            steerSpeed = 1;
        } else if (steerAngle < 70) {
            steerSpeed = 2;
        } else {
            steerSpeed = 4;
        }
    } else {
        steerSpeed = 1;
    }
    turn(steerSpeed, steerAngle > 0);
}

void goTo(Vector p, bool mayGoFaster) {
    double steerAngle = getSteerAngleTo(p);
    double factor = 1;
    double distance = vector_distanceTo(getEstimatedPosition(), p);
    if (distance < 5) factor = 1.5;
    if (abs_double(steerAngle) > getAngleTolerance() * factor) {
        turnTo(p);
    } else {
        int speed = STD_SPEED;
        if (mayGoFaster && isPositionKnown()) {
            if (distance > 30) {
                speed += 1;
            }
            if (distance > 60) {
                speed += 1;
            }
        }
        forward(speed);
    }
}

void followRoutePoint(Route *r, bool mayGoFaster) {
    if (currentRoutePoint >= r->count) {
        stop();
        return;
    }
    Vector point = r->points[currentRoutePoint];
    if (vector_distanceTo(getEstimatedPosition(), point) < ROUTE_DISTANCE_THRESHOLD) {
        currentRoutePoint++;
        followRoutePoint(r, mayGoFaster);
    } else {
        goTo(point, mayGoFaster);
    }
}

/***
 *    ########  #######  ########        ##          ###    ##    ## ######## ########
 *       ##    ##     ## ##     ##       ##         ## ##    ##  ##  ##       ##     ##
 *       ##    ##     ## ##     ##       ##        ##   ##    ####   ##       ##     ##
 *       ##    ##     ## ########        ##       ##     ##    ##    ######   ########
 *       ##    ##     ## ##              ##       #########    ##    ##       ##   ##
 *       ##    ##     ## ##              ##       ##     ##    ##    ##       ##    ##
 *       ##     #######  ##              ######## ##     ##    ##    ######## ##     ##
 */

FlowPoint calculateNearestFlowPoint(Vector point) {
    double distance = DBL_MAX;
    FlowPoint nearest = {};
    Environment *env = getCurrentEnv();
    for (int i = 0; i < env->anchorCount; i++) {
        FlowLine flowLine = env->flowLines[i];
        FlowPoint current = nearestFlowPointOnFlowLine(flowLine, point);
        double dst = vector_distanceTo(point, current.point);
        if (distance > dst) {
            distance = dst;
            nearest = current;
        }
    }
    return nearest;
}

FlowPointAndLine calculateNearestFlowRoutePoint(Vector point, FlowRoute *route) {
    // similar to calculateNearestFlowPoint
    double distance = DBL_MAX;
    FlowPoint nearest = new_FlowPoint(new_vector(-1, -1), 0, 0);
    FlowLine line = {};
    for (int i = 0; i < route->count - 1; i++) {
        FlowLine flowLine = new_FlowLine(route->points[i], route->points[i + 1]);
        FlowPoint current = nearestFlowPointOnFlowLine(flowLine, point);
        double dst = vector_distanceTo(point, current.point);
        if (distance > dst) {
            distance = dst;
            nearest = current;
            line = flowLine;
        }
    }
    return new_FlowPointAndLine(nearest, line);
}

Vector influenceByFlowPoint(const Vector position, const FlowPoint flowPoint) {
    if (flowPoint.point.x == -1) return new_vector(0, 0);

    Direction toFlowPoint = vector_directionTo(position, flowPoint.point);
    //val force = Math.abs(/*Math.cos(toFlowPoint.difference(flowPoint.direction))*/ 1) / Math.pow(flowPoint.point.distanceTo(position) / 100, 3.0);

    /*Direction *flowDirection = toFlowPoint;
    double angleDifference = direction_difference(toFlowPoint,
                                                  vector_directionTo(position, flowPoint->point));
    if (angleDifference > M_PI / 2) {
        // we are in front of the arrow, need to mirror the pull direction
        flowDirection = direction_mirrorWith(direction_invert(flowDirection), flowPoint->direction);
    }*/

    double distance = vector_distanceTo(flowPoint.point, position);
    double d = distance / flowPoint.radius;
    double relativeAngle = direction_difference(direction_invert(flowPoint.direction), toFlowPoint);
    double weight = 1 - pow(2.0, -d * d);
    //weight *= pow(cos(relativeAngle / 2), 1.0 / 2);
    weight *= (-relativeAngle) / M_PI + 1;
    Direction pullDirection = direction_weightedAverageWith(flowPoint.direction, toFlowPoint, weight);

    double size = 1 / (distance / 10 + 1);
    return vector_radial(pullDirection, size);
}

Vector influenceByFlowPointWithEnd(const Vector position, const FlowPoint flowPoint, const FlowLine flowLine) {
    if (flowPoint.point.x == -1) return new_vector(0, 0);

    Direction toFlowPoint = vector_directionTo(position, flowPoint.point);

    double d = vector_distanceTo(flowPoint.point, position) / flowPoint.radius;
    double relativeAngle = direction_difference(direction_invert(flowPoint.direction), toFlowPoint);
    double weight = 1 - pow(2.0, -d * d);
    weight *= pow(cos(relativeAngle / 2), 1.0 / 2);
    Direction pullDirection = direction_weightedAverageWith(flowPoint.direction, toFlowPoint, weight);

    double projectionDistance = vector_distanceTo(flowPoint.point, flowLine.pb.point);
    weight = toRange(projectionDistance / flowLine.pb.radius - 1, 0.0, 1.0);
    Direction final = direction_weightedAverageWith(vector_directionTo(position, flowLine.pb.point), pullDirection,
                                                    weight);

    return vector_radial(final, 1.0);
}

double forceOfBorder(double distance) {
    double a = BORDER_DISTANCE;
    double b = COEFF_K;
    double x = distance;
    return max(0.0, 2 * (exp((-x) / b) - exp((-a) / b)) / (1 - exp((-a) / b)));
}

double forceOfFlowPoint(FlowPoint flowPoint, double distance) {
    double a = flowPoint.radius;
    double b = COEFF_K;
    double x = distance;
    return max(0.0, 2 * (exp((-x) / b) - exp((-a) / b)) / (1 - exp((-a) / b)));
}

Vector influenceByBorders(const Vector position) {
    Vector v = new_vector(0, 0);
    v = vector_plus(v, vector_radial(new_Direction(0.0), forceOfBorder(position.x))); // left border
    v = vector_plus(v, vector_radial(new_Direction(M_PI / 2), forceOfBorder(position.y))); // bottom border
    v = vector_plus(v, vector_radial(new_Direction(M_PI), forceOfBorder(MAP_WIDTH - position.x))); // right border
    v = vector_plus(v,
                    vector_radial(new_Direction(M_PI * 3 / 2), forceOfBorder(MAP_HEIGHT - position.y))); // top border
    return v;
}

Vector influenceRandom(double distanceToNearestFlowPoint) {
    int offset = randn(RANDOM_VECTOR_STEP_DEVIATION * 2) - RANDOM_VECTOR_STEP_DEVIATION; // offset in degrees
    lastRandomDirection = direction_plus(lastRandomDirection, direction_fromDegrees(offset));
    double randomForce = max(0, -(1 / (distanceToNearestFlowPoint - 1) * 3) + 1);
    return vector_radial(lastRandomDirection, getCurrentEnv()->randomnessSize * randomForce);
}

Vector influenceRandomNearSuperobject(double distanceToSuperobject) {
    int offset = randn(RANDOM_VECTOR_STEP_DEVIATION * 2) - RANDOM_VECTOR_STEP_DEVIATION; // offset in degrees
    lastRandomDirection = direction_plus(lastRandomDirection, direction_fromDegrees(offset));
    double randomForce = max(0, 4.8 / (distanceToSuperobject + 2) - 0.4);
    return vector_radial(lastRandomDirection, randomForce);
}

Vector influenceByDiscreteFlowPoint(const Vector position, FlowPoint flowPoint) {
    return vector_radial(flowPoint.direction,
                         DISCRETE_FLOWPOINT_FORCE *
                         forceOfFlowPoint(flowPoint, vector_distanceTo(position, flowPoint.point))
    );
}

Vector influenceByDiscreteFlowPoints(const Vector position) {
    Vector sum = new_vector(0, 0);
    Environment *env = getCurrentEnv();
    for (int i = 0; i < env->flowPointCount; i++) {
        sum = vector_plus(sum, influenceByDiscreteFlowPoint(position, env->flowPoints[i]));
    }
    return sum;
}

Vector influenceByRoute(const Vector position, FlowRoute *route) {
    FlowPointAndLine result = calculateNearestFlowRoutePoint(position, route);
    Vector lineBPoint = result.line.pb.point;
    Vector lastRoutePoint = route->points[route->count - 1].point;
    if (route->withEnd && lineBPoint.x == lastRoutePoint.x && lineBPoint.y == lastRoutePoint.y) {
        return influenceByFlowPointWithEnd(position, result.point, result.line);
    }
    return influenceByFlowPoint(position, result.point);
}

Vector influenceByRoutes(const Vector position) {
    Vector sum = new_vector(0, 0);
    Environment *env = getCurrentEnv();
    for (int i = 0; i < env->routeCount; i++) {
        sum = vector_plus(sum, influenceByRoute(position, &env->flowRoutes[i]));
    }
    return sum;
}

Vector calculateMoveVector(Vector position) {
    FlowPoint nearestFlowPoint = calculateNearestFlowPoint(position);
    Vector infFlowPoint = influenceByFlowPoint(position, nearestFlowPoint);
    Vector infBorders = influenceByBorders(position);
    Vector infRandom = influenceRandom(vector_distanceTo(position, nearestFlowPoint.point));
    Vector infDiscretes = influenceByDiscreteFlowPoints(position);
    Vector infRoutes = influenceByRoutes(position);
    return vector_plus(vector_plus(vector_plus(vector_plus(infFlowPoint, infBorders), infRandom), infDiscretes),
                       infRoutes);
}

Vector calculateSuperobjectFlowlineMoveVector(Vector position) {
    FlowLine flowLine = superobjectFlowLine;
    FlowPoint flowPoint = nearestFlowPointOnFlowLine(flowLine, position);
    Direction toFlowPoint = vector_directionTo(position, flowPoint.point);

    double distance = vector_distanceTo(flowPoint.point, position);
    double d = distance / flowPoint.radius;
    double relativeAngle = direction_difference(direction_invert(flowPoint.direction), toFlowPoint);
    double weight = 1 - pow(2.0, -d * d);
    weight *= pow(cos(relativeAngle / 2), 1.0 / 2);
    Direction pullDirection = direction_weightedAverageWith(flowPoint.direction, toFlowPoint, weight);

    double projectionDistance = vector_distanceTo(flowPoint.point, flowLine.pb.point);
    weight = toRange(projectionDistance / flowLine.pb.radius - 1, 0, 1);
    Direction target = direction_weightedAverageWith(vector_directionTo(position, flowLine.pb.point), pullDirection,
                                                     weight);

    return vector_plus(vector_radial(target, 1.0), influenceRandom(distance));
}

/***
 *    ########  ########   #######   ######   ########     ###    ##     ##
 *    ##     ## ##     ## ##     ## ##    ##  ##     ##   ## ##   ###   ###
 *    ##     ## ##     ## ##     ## ##        ##     ##  ##   ##  #### ####
 *    ########  ########  ##     ## ##   #### ########  ##     ## ## ### ##
 *    ##        ##   ##   ##     ## ##    ##  ##   ##   ######### ##     ##
 *    ##        ##    ##  ##     ## ##    ##  ##    ##  ##     ## ##     ##
 *    ##        ##     ##  #######   ######   ##     ## ##     ## ##     ##
 */

void init() {
    if (!initialized) {
        initialized = true;

        rnd_state_o = *new_rnd_state();

        _init_values();
        _init_flowlines();

        lastRandomDirection = new_Direction(0);

        lastPosition = estimatedPosition = getCurrentPosition();
        lastDirection = estimatedDirection = getCurrentDirection();

        setBottomEnv(1);
    }
}

int doStates() {
    Vector position = getEstimatedPosition();

    // Collecting
    if (collectingTime > 0 && (mustRemainCollecting || canCollect())) {
        collectingTime--;
        if (collectingTime < 10 && canCollect()) mustRemainCollecting = false;
        return ACTION_COLLECTING;
    }

    // Depositing
    if (depositingTime > 0) {
        depositingTime--;
        return ACTION_DEPOSITING;
    }

    // Collecting
    if (canCollect() && shouldCollect()) {
        collectingTime = 38;
        mustRemainCollecting = true;
        registerCollect();
        if (isViolet()) stopFollowingSuperobject();
        if (wantsDeposit()) {
            depositEnvIndex = 0;
        }
        stop();
        return ACTION_COLLECT;
    }
    mustRemainCollecting = false;
    collectingTime = 0;

    // Reverse
    if (reversingTime > 0) {
        reversingTime--;
        forward(-STD_SPEED);
        return ACTION_REVERSING;
    }

    // Avoid obstacle
    if (avoidingObstacleTime > 0) {
        avoidingObstacleTime--;
        return ACTION_OBSTACLE_AVOIDING;
    }
    if (shouldAvoidObstacle()) {
        avoidingObstacleTime = 15;
        if (seesObstacleLeft() || isYellowLeft()) {
            move(-2, -1);
        } else if (seesObstacleRight() || isYellowRight()) {
            move(-1, -2);
        } else {
            move(-2, -2);
        }
        return ACTION_OBSTACLE_AVOID;
    }

    // Deposit
    if (shouldDeposit() && canDeposit()) {
        depositingTime = DEPOSITING_TIME;
        reversingTime = 8;
        registerDeposit();
        depositEnvIndex = NONE;
        setBottomEnv(1);
        stop();
        return ACTION_DEPOSIT;
    }
    if (shouldDeposit() && (isOrangeLeft() || isOrangeRight())) {
        if (isOrangeLeft()) {
            move(0, 2); // go to left
        } else {
            move(2, 0); // go to right
        }
        return ACTION_ADJUST_FOR_DEPOSIT;
    }

    // Superobject Vision
    if (!isFollowingSuperobject) {
        generateSuperobjectRoute();
    }
    if (isFollowingSuperobject) {
        if (Time - lastDecidedForSuperobject > 60) {
            stopFollowingSuperobject();
        }
        Vector moveVector = calculateSuperobjectFlowlineMoveVector(position);
        Vector target = vector_plus(position, moveVector);
        steerTo(target);
        return ACTION_FOLLOW_SUPEROBJECT;
    }

    // Follow route
    Route *route = getCurrentRoute();
    if (route != NULL) {
        followRoutePoint(route, true);
        return ACTION_FOLLOW_ROUTE;
    }

    // Follow flow
    Vector moveVector = calculateMoveVector(position);
    Vector target = vector_plus(position, moveVector);
    //turnTo(target);
    steerTo(target);

    return ACTION_NORMAL;
}

void switchEnvIfNeeded() {
    Vector pos = getEstimatedPosition();
    if (isInRect(pos, 107, 242, 0, 270)) {
        setBottomEnv(1);
    } else if (isInRect(pos, 200, 0, 240, 60)) {
        setBottomEnv(2);
    }
    if ((Time >= 480 - 25) && shouldDeposit()) {
        depositEnvIndex = 0;
    }
}

void Game1() {

    init();

    if (SuperDuration > 0) {
        SuperDuration--;
    } else if (Duration > 0) {
        Duration--;
    }
    ticks += 1;

    registerSuperobject();
    observePosition();
    switchEnvIfNeeded();

    lastState = doStates();
    debug1 = lastState;

    if (depositingTime > 0) {
        LED_1 = 2;
    } else if (collectingTime > 0) {
        LED_1 = 1;
    } else {
        LED_1 = 0;
    }

}



void Game0() {

    if (SuperDuration > 0) {
        SuperDuration--;
    } else if (Time >= 180 && Time <= 200) {
        SuperDuration = 49;
        Duration = 0;
        CurAction = 17;
    } else if (Duration > 0) {
        Duration--;
    } else if (start == 0) {
        Duration = 14;
        CurAction = 1;
    } else if (start == 1) {
        Duration = 39;
        CurAction = 2;
    } else if (CSLeft_R >= 200 && CSLeft_R <= 255 && CSLeft_G >= 150 && CSLeft_G <= 205 && CSLeft_B >= 0 &&
               CSLeft_B <= 50 && CSRight_R >= 200 && CSRight_R <= 255 && CSRight_G >= 150 && CSRight_G <= 205 &&
               CSRight_B >= 0 && CSRight_B <= 50 && (LoadedObjects > 0)) {
        Duration = 59;
        CurAction = 3;
    } else if (justdeposited == 1) {
        Duration = 0;
        CurAction = 4;
    } else if (justdeposited == 2 && Compass < 250 && Compass > 200) {
        Duration = 9;
        CurAction = 5;
    } else if (CSRight_R >= 200 && CSRight_R <= 255 && CSRight_G >= 150 && CSRight_G <= 205 && CSRight_B >= 0 &&
               CSRight_B <= 50 && (goingtodeposit < 10)) {
        Duration = 0;
        CurAction = 6;
    } else if (CSLeft_R >= 200 && CSLeft_R <= 255 && CSLeft_G >= 150 && CSLeft_G <= 205 && CSLeft_B >= 0 &&
               CSLeft_B <= 50 && (goingtodeposit < 10)) {
        Duration = 0;
        CurAction = 7;
    } else if (CSRight_R >= 190 && CSRight_R <= 255 && CSRight_G >= 210 && CSRight_G <= 255 && CSRight_B >= 0 &&
               CSRight_B <= 40 && ((LoadedObjects > 0 || (Compass < 170 || Compass > 290)) && goingtodeposit < 2)) {
        Duration = 3;
        CurAction = 8;
    } else if (CSLeft_R >= 190 && CSLeft_R <= 255 && CSLeft_G >= 210 && CSLeft_G <= 255 && CSLeft_B >= 0 &&
               CSLeft_B <= 40 && ((LoadedObjects > 0 || (Compass < 170 || Compass > 290)) && goingtodeposit < 2)) {
        Duration = 1;
        CurAction = 9;
    } else if (foundDeposit == 1) {
        Duration = 0;
        CurAction = 10;
    } else if (goingtodeposit == 2 && Compass < 50 && Compass > 30) {
        Duration = 0;
        CurAction = 11;
    } else if (goingtodeposit == 3) {
        Duration = 0;
        CurAction = 12;
    } else if (//red//
            (((CSLeft_R >= 200 && CSLeft_R <= 255 && CSLeft_G >= 20 && CSLeft_G <= 50 && CSLeft_B >= 20 &&
               CSLeft_B <= 50)
              ||
              (CSRight_R >= 200 && CSRight_R <= 255 && CSRight_G >= 20 && CSRight_G <= 50 && CSRight_B >= 20 &&
               CSRight_B <= 50))

             ||

             //cyan//
             ((CSLeft_R >= 15 && CSLeft_R <= 50 && CSLeft_G >= 220 && CSLeft_G <= 255 && CSLeft_B >= 220 &&
               CSLeft_B <= 255)
              ||
              (CSRight_R >= 15 && CSRight_R <= 50 && CSRight_G >= 220 && CSRight_G <= 255 && CSRight_B >= 220 &&
               CSRight_B <= 255))

             ||


             //black//
             ((CSLeft_R >= 14 && CSLeft_R <= 40 && CSLeft_G >= 14 && CSLeft_G <= 40 && CSLeft_B >= 14 && CSLeft_B <= 40)
              ||
              (CSRight_R >= 14 && CSRight_R <= 40 && CSRight_G >= 14 && CSRight_G <= 40 && CSRight_B >= 14 &&
               CSRight_B <= 40)))


            &&
            (LoadedObjects < 6)
            ) {
        Duration = 49;
        CurAction = 13;
    } else if ((US_Left > 1 && US_Left < 10)) {
        Duration = 1;
        CurAction = 14;
    } else if ((US_Right > 1 && US_Right < 10)) {
        Duration = 1;
        CurAction = 15;
    } else if (US_Front >= 0 && US_Front <= 9) {
        Duration = 2;
        CurAction = 16;
    } else if (goingtodeposit == 1) {
        Duration = 0;
        CurAction = 18;
    } else if (goingtodeposit == 0) {
        Duration = 0;
        CurAction = 19;
    }
    switch (CurAction) {
        case 1:
            WheelLeft = 1;
            WheelRight = -1;
            LED_1 = 0;
            MyState = 0;
            start = 1;

            break;
        case 2:
            WheelLeft = 4;
            WheelRight = 4;
            LED_1 = 0;
            MyState = 0;
            start = 2;

            break;
        case 3:
            WheelLeft = 0;
            WheelRight = 0;
            LED_1 = 2;
            MyState = 0;
            goingtodeposit = 0;

            justdeposited = 1;

            foundDeposit = 0;

            if (Duration == 1) { LoadedObjects = 0; }
            break;
        case 4:
            WheelLeft = 1;
            WheelRight = -1;
            LED_1 = 0;
            MyState = 0;
            justdeposited = 2;

            goingtodeposit = 99;

            break;
        case 5:
            WheelLeft = 3;
            WheelRight = 3;
            LED_1 = 0;
            MyState = 0;
            justdeposited = 0;

            goingtodeposit = 0;

            break;
        case 6:
            WheelLeft = 0;
            WheelRight = 0;
            LED_1 = 0;
            MyState = 0;
            if (LoadedObjects == 0 && goingtodeposit == 0) {
                WheelLeft = -1;

                WheelRight = -4;

            } else if (LoadedObjects > 0) {
                WheelLeft = 1;

                WheelRight = 0;

            }
            break;
        case 7:
            WheelLeft = 0;
            WheelRight = 0;
            LED_1 = 0;
            MyState = 0;
            if (LoadedObjects == 0 && goingtodeposit == 0) {
                WheelLeft = -2;

                WheelRight = -4;

            } else if (LoadedObjects > 0) {
                WheelLeft = 0;

                WheelRight = 1;

            }

            break;
        case 8:
            WheelLeft = 0;
            WheelRight = 0;
            LED_1 = 0;
            MyState = 0;
            if (goingtodeposit == 0) {
                WheelLeft = 1;

                WheelRight = -2;

            } else if (goingtodeposit == 1 && (Compass < 40 || Compass > 350)) {
                WheelLeft = 0;

                WheelRight = 1;

                foundDeposit = 1;

            } else {
                WheelLeft = 1;

                WheelRight = -2;

            }


            break;
        case 9:
            WheelLeft = 0;
            WheelRight = 0;
            LED_1 = 0;
            MyState = 0;
            if (goingtodeposit == 0) {
                WheelLeft = 1;

                WheelRight = -2;

            } else if ((Compass >= 0 && Compass < 160) || (Compass <= 360 && Compass > 330)) {
                WheelLeft = 0;

                WheelRight = -2;

            } else if (goingtodeposit == 1) {
                WheelLeft = 0;

                WheelRight = -2;

            }
            break;
        case 10:
            WheelLeft = 0;
            WheelRight = 1;
            LED_1 = 0;
            MyState = 0;
            goingtodeposit = 2;

            foundDeposit = 0;

            break;
        case 11:
            WheelLeft = 0;
            WheelRight = 0;
            LED_1 = 0;
            MyState = 0;
            goingtodeposit = 3;

            foundDeposit = 0;

            break;
        case 12:
            WheelLeft = 1;
            WheelRight = 1;
            LED_1 = 0;
            MyState = 0;
            goingtodeposit = 1;

            break;
        case 13:
            WheelLeft = 0;
            WheelRight = 0;
            LED_1 = 1;
            MyState = 0;
            if (Duration == 1) LoadedObjects++;
            if (Duration < 6) {
                WheelLeft = 2;
                WheelRight = 2;
            }
            break;
        case 14:
            WheelLeft = 0;
            WheelRight = 0;
            LED_1 = 0;
            MyState = 0;
            if (goingtodeposit == 0) {
                WheelLeft = 2;

                WheelRight = -2;

            } else if (Compass > 180 && Compass < 270) {
                WheelLeft = 2;

                WheelRight = -2;

            } else {
                WheelLeft = 0;

                WheelRight = -2;

            }
            break;
        case 15:
            WheelLeft = 0;
            WheelRight = 0;
            LED_1 = 0;
            MyState = 0;
            if (goingtodeposit == 0) {
                WheelLeft = 1;

                WheelRight = -2;

            } else {
                WheelLeft = 0;

                WheelRight = -2;

            }
            break;
        case 16:
            WheelLeft = 0;
            WheelRight = 0;
            LED_1 = 0;
            MyState = 0;
            if (goingtodeposit == 0) {
                WheelLeft = 1;

                WheelRight = -2;

            } else {
                WheelLeft = 0;

                WheelRight = -1;

            }
            break;
        case 17:
            WheelLeft = 0;
            WheelRight = 0;
            LED_1 = 0;
            MyState = 0;
            Teleport = 1;
            LoadedObjects = 0;
            WheelLeft = 0;
            WheelRight = 0;
            LED_1 = 0;
            Duration = 0;
            SuperDuration = 0;
            break;
        case 18:
            WheelLeft = 0;
            WheelRight = 0;
            LED_1 = 0;
            MyState = 0;
            if (Compass < 330 && Compass >= 310) {
                WheelLeft = -1;

                WheelRight = 1;

            } else if (Compass < 310 && Compass >= 220) {
                WheelLeft = 3;

                WheelRight = 2;

            } else if (Compass < 220 && Compass > 160) {

                WheelLeft = 2;

                WheelRight = 2;

            } else {
                WheelLeft = 1;

                WheelRight = 2;

            }
            break;
        case 19:
            WheelLeft = 2;
            WheelRight = 2;
            LED_1 = 0;
            MyState = 0;
            if (LoadedObjects > 3) {
                goingtodeposit = 1;

            } else {
                goingtodeposit = 0;

            }


            break;
        default:
            break;
    }

}

/***
 *     ######   #######   ######  ########     ###     ######  ########
 *    ##    ## ##     ## ##    ## ##     ##   ## ##   ##    ## ##
 *    ##       ##     ## ##       ##     ##  ##   ##  ##       ##
 *    ##       ##     ##  ######  ########  ##     ## ##       ######
 *    ##       ##     ##       ## ##        ######### ##       ##
 *    ##    ## ##     ## ##    ## ##        ##     ## ##    ## ##
 *     ######   #######   ######  ##        ##     ##  ######  ########
 *
 * COSPACE Stuff (do not edit)
 */

DLL_EXPORT void SetGameID(int GameID) {
    CurGame = GameID;
    bGameEnd = 0;
}

DLL_EXPORT int GetGameID() {
    return CurGame;
}

//Only Used by CsBot Dance Platform
DLL_EXPORT int IsGameEnd() {
    return bGameEnd;
}

#ifndef CSBOT_REAL

char info[1024];

DLL_EXPORT char *GetDebugInfo() {
    sprintf(info,
            "env=%d;debug1=%d;debug2=%d;lastState=%d;estPosX=%d;estPosY=%d;estDir=%d;Duration=%d;SuperDuration=%d;bGameEnd=%d;CurAction=%d;CurGame=%d;SuperObj_Num=%d;SuperObj_X=%d;SuperObj_Y=%d;Teleport=%d;LoadedObjects=%d;US_Front=%d;US_Left=%d;US_Right=%d;CSLeft_R=%d;CSLeft_G=%d;CSLeft_B=%d;CSRight_R=%d;CSRight_G=%d;CSRight_B=%d;PositionX=%d;PositionY=%d;TM_State=%d;Compass=%d;Time=%d;WheelLeft=%d;WheelRight=%d;LED_1=%d;MyState=%d;",
            getCurrentEnvIndex(), debug1, debug2, lastState, estPosX(), estPosY(), estDir(), Duration, SuperDuration,
            bGameEnd, CurAction,
            CurGame, SuperObj_Num, SuperObj_X,
            SuperObj_Y,
            Teleport,
            LoadedObjects, US_Front, US_Left, US_Right, CSLeft_R, CSLeft_G, CSLeft_B, CSRight_R, CSRight_G,
            CSRight_B,
            PX, PY, TM_State, Compass, Time, WheelLeft, WheelRight, LED_1, MyState);
    return info;
}

DLL_EXPORT char *GetTeamName() {
    return "Talentum";
}

DLL_EXPORT int GetCurAction() {
    return CurAction;
}

//Only Used by CsBot Rescue Platform
DLL_EXPORT int GetTeleport() {
    return Teleport;
}

//Only Used by CsBot Rescue Platform
DLL_EXPORT void SetSuperObj(int X, int Y, int num) {
    SuperObj_X = X;
    SuperObj_Y = Y;
    SuperObj_Num = num;
}
//Only Used by CsBot Rescue Platform
DLL_EXPORT void GetSuperObj(int *X, int *Y, int *num) {
    *X = SuperObj_X;
    *Y = SuperObj_Y;
    *num = SuperObj_Num;
}

#endif ////CSBOT_REAL

DLL_EXPORT void SetDataAI(volatile int *packet, const volatile int *AI_IN) {

    int sum = 0;

    US_Front = AI_IN[0];
    packet[0] = US_Front;
    sum += US_Front;
    US_Left = AI_IN[1];
    packet[1] = US_Left;
    sum += US_Left;
    US_Right = AI_IN[2];
    packet[2] = US_Right;
    sum += US_Right;
    CSLeft_R = AI_IN[3];
    packet[3] = CSLeft_R;
    sum += CSLeft_R;
    CSLeft_G = AI_IN[4];
    packet[4] = CSLeft_G;
    sum += CSLeft_G;
    CSLeft_B = AI_IN[5];
    packet[5] = CSLeft_B;
    sum += CSLeft_B;
    CSRight_R = AI_IN[6];
    packet[6] = CSRight_R;
    sum += CSRight_R;
    CSRight_G = AI_IN[7];
    packet[7] = CSRight_G;
    sum += CSRight_G;
    CSRight_B = AI_IN[8];
    packet[8] = CSRight_B;
    sum += CSRight_B;
    PX = AI_IN[9];
    packet[9] = PX;
    sum += PX;
    PY = AI_IN[10];
    packet[10] = PY;
    sum += PY;
    TM_State = AI_IN[11];
    packet[11] = TM_State;
    sum += TM_State;
    Compass = AI_IN[12];
    packet[12] = Compass;
    sum += Compass;
    Time = AI_IN[13];
    packet[13] = Time;
    sum += Time;
    packet[14] = sum;

}

DLL_EXPORT void GetCommand(int *AI_OUT) {
    AI_OUT[0] = WheelLeft;
    AI_OUT[1] = WheelRight;
    AI_OUT[2] = LED_1;
    AI_OUT[3] = MyState;
}

DLL_EXPORT void OnTimer() {
    switch (CurGame) {
        case 9:
            break;
        case 10:
            WheelLeft = 0;
            WheelRight = 0;
            LED_1 = 0;
            MyState = 0;
            break;
        case 0:
            Game0();
            break;
        case 1:
            Game1();
            break;
        default:
            break;
    }
}

#pragma clang diagnostic pop
