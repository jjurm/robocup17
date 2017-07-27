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
char AI_MyID[2] = {'0', '2'};

double toRange(double value, double min, double max);

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

Vector *new_vector(double xa, double ya) {
    Vector *p = malloc(sizeof(Vector));
    p->x = xa;
    p->y = ya;
    return p;
}

typedef struct {
    int xa, xb, ya, yb;
} Area;

Area *new_area(int xa, int ya, int xb, int yb) {
    Area *o = malloc(sizeof(Area));
    o->xa = xa;
    o->xb = xb;
    o->ya = ya;
    o->yb = yb;
    return o;
}

typedef double Direction;

Direction *new_Direction(double value) {
    Direction *o = malloc(sizeof(Direction));
    *o = direction_normalize(value);
    return o;
}

typedef struct {
    Vector *point;
    double radius;
} Anchor;

Anchor *new_Anchor(Vector *point, double radius) {
    Anchor *o = malloc(sizeof(Anchor));
    o->point = point;
    o->radius = radius;
    return o;
}

typedef struct {
    Vector *point;
    double radius;
    Direction *direction;
} FlowPoint;

FlowPoint *new_FlowPoint(Vector *point, double radius, Direction *direction) {
    FlowPoint *o = malloc(sizeof(FlowPoint));
    o->point = point;
    o->radius = radius;
    o->direction = direction;
    return o;
}

typedef struct {
    Anchor *pa;
    Anchor *pb;
} FlowLine;

FlowLine *new_FlowLine(Anchor *pa, Anchor *pb) {
    FlowLine *o = malloc(sizeof(FlowLine));
    o->pa = pa;
    o->pb = pb;
    return o;
}

typedef struct {
    double randomnessSize;
    int anchorCount;
    Anchor *anchors[50];
    FlowLine *flowLines[50];
    int flowPointCount;
    FlowPoint *flowPoints[10];
} Environment;

typedef struct {
    int count;
    Vector *points[50];
} Route;


//========== VECTOR ==========

Vector *vector_radial(const Direction *direction, double size) {
    return new_vector(
            cos((*direction)) * size,
            sin((*direction)) * size
    );
}

double vector_size(Vector *A) {
    return sqrt(pow(A->x, 2) + pow(A->y, 2));
}

Vector *vector_vectorTo(const Vector *A, const Vector *B) {
    return new_vector(B->x - A->x, B->y - A->y);
}

double vector_distanceTo(const Vector *A, const Vector *B) {
    return vector_size(vector_vectorTo(A, B));
}

Direction *vector_direction(Vector *A) {
    return new_Direction(atan2(A->y, A->x));
}

Direction *vector_directionTo(const Vector *A, Vector *B) {
    return vector_direction(vector_vectorTo(A, B));
}

Vector *vector_plus(Vector *A, Vector *B) {
    return new_vector(A->x + B->x, A->y + B->y);
}

Vector *vector_minus(Vector *A, Vector *B) {
    return new_vector(A->x - B->x, A->y - B->y);
}

Vector *vector_multiply(Vector *A, double k) {
    return new_vector(A->x * k, A->y * k);
}

Vector *vector_invert(Vector *A) {
    return new_vector(-A->x, -A->y);
}

//========== AREA ==========

//========== DIRECTION ==========

Direction *direction_fromDegrees(double degrees) {
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

double direction_differenceTo(const Direction *this, const Direction *direction) {
    double target = *direction;
    if (target < (*this)) target += 2 * M_PI;
    return target - (*this);
}

double direction_difference(const Direction *this, const Direction *direction) {
    return min(
            direction_differenceTo(this, direction),
            direction_differenceTo(direction, this)
    );
}

Direction *direction_plus(const Direction *this, const Direction *direction) {
    return new_Direction((*this) + (*direction));
}

Direction *direction_minus(const Direction *this, const Direction *direction) {
    return new_Direction((*this) - (*direction));
}

Direction *direction_invert(const Direction *this) {
    return new_Direction((*this) + M_PI);
}

double direction_degrees(const Direction *this) {
    return (*this) * 180 / M_PI;
}

Direction *direction_mirrorWith(const Direction *this, const Direction *axis) {
    return new_Direction(2 * (*axis) - (*this));
}

Direction *direction_weightedAverageWith(Direction *this, Direction *direction, double weight) {
    return vector_direction(vector_plus(vector_radial(this, 1 - weight), vector_radial(direction, weight)));
}

Direction *direction_averageWith(Direction *this, Direction *direction) {
    return direction_weightedAverageWith(this, direction, 0.5);
}

//========== ANCHOR ==========

//========== FLOWPOINT ==========

//========== FLOWLINE ==========

FlowPoint *nearestFlowPoint(FlowLine *this, Vector *point) {
    Vector *aToP = vector_vectorTo(this->pa->point, point);
    Vector *aToB = vector_vectorTo(this->pa->point, this->pb->point);
    double atb2 = pow(aToB->x, 2) + pow(aToB->y, 2);
    double atp_dot_atb = aToP->x * aToB->x + aToP->y * aToB->y;

    double t = toRange(atp_dot_atb / atb2, 0, 1);

    return new_FlowPoint(vector_plus(this->pa->point, vector_multiply(aToB, t)),
                         t * this->pb->radius + (1 - t) * this->pa->radius,
                         vector_direction(aToB));
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
#define ACTION_COLLECT 20
#define ACTION_COLLECTING 21
#define ACTION_OBSTACLE_AVOID 30
#define ACTION_OBSTACLE_AVOIDING 31
#define ACTION_FOLLOW_ROUTE 41
#define ACTION_NORMAL 1

int MIN_DEP_LOADED_OBJECTS = 4;
int STD_SPEED = 2;
double STD_ANGLE_TOLERANCE = 8 * M_PI / 180;
int AVOIDING_BORDER_TIME = 20;
int DEPOSITING_TIME = 40;
int RANDOM_COORDINATES_PADDING = 30;
int US_DISTANCE = 3;
int ROUTE_DISTANCE_THRESHOLD = 8;

double BORDER_DISTANCE = 20;
double COEFF_K = 16;
double DISCRETE_FLOWPOINT_FORCE = 2.0;
int RANDOM_VECTOR_STEP_DEVIATION = 20; // degrees
double RANDOM_VECTOR_SIZE = 0.4;

//========== PROGRAM variables ==========
bool initialized = false;
int ticks = 0;

// Superobjects
int superobjectCount = 0;
Vector *superobjects[20];
Vector *lastSuperobjectRegistered;

//========== STATE variables ==========
int collectingTime = 0;
bool isLongerCollecting = false;
int currentArea = 0;
int currentCheckpoint = 0;
int avoidingObstacleTime = 0;
int avoidingBorderTime = 0;
Vector *avoidingBorderPos;
int depositingTime = 0;
int currentEnvironment = 0;
int currentRoute = 0;
int currentRoutePoint = 0;

// moving and position
Vector *lastPosition;
Direction *lastDirection;
Vector *estimatedPosition;
Direction *estimatedDirection;
int motorLeft;
int motorRight;
Direction *lastRandomDirection;

//========== TEMPORARY variables ==========
int lastState = 0;
int debug1 = -777;
int debug2 = -777;

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

#define AREAS_COUNT 1
Area *AREAS[AREAS_COUNT];
int _index_area = 0;

void _area(int x1, int y1, int x2, int y2) {
    AREAS[_index_area++] = new_area(x1, y1, x2, y2);
}

void _init_areas() {
    //#################### AREA ####################
    _area(1, 2, 3, 4);
}

// ========== ENVIRONMENTS ==========
#define ENVIRONMENT_COUNT 1
Environment ENVIRONMENTS[] = {
        { // STANDARD
                0.6, // randomness
                0, {}, {}, 0, {}
        }
};

Environment *getCurrentEnv() {
    if (currentEnvironment == NONE) return NULL;
    return &ENVIRONMENTS[currentEnvironment];
}

void _anchor(int env, int x, int y, int radius) {
    Environment *e = &ENVIRONMENTS[env];
    e->anchors[e->anchorCount++] = new_Anchor(new_vector(x, y), radius);
}

void _flowPoint(int env, int x, int y, int radius, int direction) {
    Environment *e = &ENVIRONMENTS[env];
    e->flowPoints[e->flowPointCount++] = new_FlowPoint(new_vector(x, y), radius, direction_fromDegrees(direction));
}

void _init_flowlines() {
    for (int ei = 0; ei < ENVIRONMENT_COUNT; ei++) {
        Environment *e = &ENVIRONMENTS[ei];
        for (int i = 0; i < e->anchorCount; i++) {
            Anchor *aa = e->anchors[i];
            Anchor *ab = e->anchors[(i + 1) % e->anchorCount];
            e->flowLines[i] = new_FlowLine(aa, ab);
        }
    }
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

void _init_values() {
    //####################VALUES####################
    _anchor(E_A, 88, 58, 20);
    _anchor(E_A, 102, 145, 10);
    _anchor(E_A, 90, 200, 35);
    _anchor(E_A, 197, 244, 4);
    _anchor(E_A, 280, 242, 4);
    _anchor(E_A, 292, 134, 20);
    _anchor(E_A, 337, 90, 4);
    _anchor(E_A, 337, 24, 4);
    _anchor(E_A, 190, 26, 20);

    _flowPoint(E_A, 10, 165, 60, 80);

    // routes: put the list of points here
    int r = 6;
    int r2 = 5;
    _routePoint(0, 18, 72 + r);//1
    _routePoint(0, 74 + r, 70);
    _routePoint(0, 72, 106 + r);
    _routePoint(0, 107 + r, 102);
    _routePoint(0, 110, 72 - r);
    _routePoint(0, 164 + r, 71);
    _routePoint(0, 159, 135 + r);
    _routePoint(0, 100 - r, 135);
    _routePoint(0, 100, 167 + r);
    _routePoint(0, 193 + r, 175);//10
    _routePoint(0, 188, 194 + r);
    _routePoint(0, 71 - r, 198);
    _routePoint(0, 41 - r2, 246 + r2);
    _routePoint(0, 250 + r, 250);//before 15
    _routePoint(0, 249, 71 - r);//16
    _routePoint(0, 162 - r2, 16);
    _routePoint(0, 235 + r, 45);//before 18
    _routePoint(0, 281 + r, 16 - r2);//18
    _routePoint(0, 338 + r2, 102 + r);
    _routePoint(0, 307, 170 + r);//before END
    _routePoint(0, 339, 247 + r);
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

double toRange(double value, double min, double max) {
    return min(max(min, value), max);
}

double abs_double(double value) {
    return value >= 0 ? value : -value;
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
    return (((CSRight_R > 188 - 10) && (CSRight_R < 217 + 10)) && ((CSRight_G > 105 - 10) && (CSRight_G < 121 + 10)) &&
            ((CSRight_B > 52 - 10) && (CSRight_B < 61 + 10)));
}

bool isOrangeLeft() {
    return (((CSLeft_R > 188 - 10) && (CSLeft_R < 217 + 10)) && ((CSLeft_G > 105 - 10) && (CSLeft_G < 121 + 10)) &&
            ((CSLeft_B > 52 - 10) && (CSLeft_B < 61 + 10)));
}

bool isOrange() { return (isOrangeLeft() || isOrangeRight()); }

bool isGreyRight() {
    return (((CSRight_R > 140 - 10) && (CSRight_R < 161 + 10)) && ((CSRight_G > 145 - 10) && (CSRight_G < 166 + 10)) &&
            ((CSRight_B > 189 - 10) && (CSRight_B < 215 + 10)));
}

bool isGreyLeft() {
    return (((CSLeft_R > 140 - 10) && (CSLeft_R < 161 + 10)) && ((CSLeft_G > 145 - 10) && (CSLeft_G < 166 + 10)) &&
            ((CSLeft_B > 189 - 10) && (CSLeft_B < 215 + 10)));
}

bool isGrey() { return (isGreyLeft() || isGreyRight()); }

bool isDarkredRight() {
    return (((CSRight_R > 151 - 10) && (CSRight_R < 176 + 10)) && ((CSRight_G > 0 - 10) && (CSRight_G < 0 + 10)) &&
            ((CSRight_B > 0 - 10) && (CSRight_B < 0 + 10)));
}

bool isDarkredLeft() {
    return (((CSLeft_R > 151 - 10) && (CSLeft_R < 176 + 10)) && ((CSLeft_G > 0 - 10) && (CSLeft_G < 0 + 10)) &&
            ((CSLeft_B > 0 - 10) && (CSLeft_B < 0 + 10)));
}

bool isDarkred() { return (isDarkredLeft() || isDarkredRight()); }

bool isYellowRight() {
    return (((CSRight_R > 202 - 10) && (CSRight_R < 235 + 10)) && ((CSRight_G > 215 - 10) && (CSRight_G < 248 + 10)) &&
            ((CSRight_B > 1 - 10) && (CSRight_B < 9 + 10)));
}

bool isYellowLeft() {
    return (((CSLeft_R > 202 - 10) && (CSLeft_R < 235 + 10)) && ((CSLeft_G > 215 - 10) && (CSLeft_G < 248 + 10)) &&
            ((CSLeft_B > 1 - 10) && (CSLeft_B < 9 + 10)));
}

bool isYellow() { return (isYellowLeft() || isYellowRight()); }

bool isBlackRight() {
    return (((CSRight_R > 30 - 10) && (CSRight_R < 39 + 10)) && ((CSRight_G > 30 - 10) && (CSRight_G < 39 + 10)) &&
            ((CSRight_B > 30 - 10) && (CSRight_B < 39 + 10)));
}

bool isBlackLeft() {
    return (((CSLeft_R > 30 - 10) && (CSLeft_R < 39 + 10)) && ((CSLeft_G > 30 - 10) && (CSLeft_G < 39 + 10)) &&
            ((CSLeft_B > 30 - 10) && (CSLeft_B < 39 + 10)));
}

bool isBlack() { return (isBlackLeft() || isBlackRight()); }

bool isDarkBlueRight() {
    return (((CSRight_R > 1 - 10) && (CSRight_R < 1 + 10)) && ((CSRight_G > 150 - 10) && (CSRight_G < 170 + 10)) &&
            ((CSRight_B > 255 - 10) && (CSRight_B < 255 + 10)));
}

bool isDarkBlueLeft() {
    return (((CSLeft_R > 1 - 10) && (CSLeft_R < 1 + 10)) && ((CSLeft_G > 150 - 10) && (CSLeft_G < 170 + 10)) &&
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


// =========== AREAS ==========

Vector *randomCoordinates(int arean) {
    Area *a = AREAS[arean];
    int x = randn(a->xb - a->xa - 2 * RANDOM_COORDINATES_PADDING) + a->xa + RANDOM_COORDINATES_PADDING;
    int y = randn(a->yb - a->ya - 2 * RANDOM_COORDINATES_PADDING) + a->ya + RANDOM_COORDINATES_PADDING;
    return new_vector(x, y);
}

// =========== CHECKS ==========

bool canCollect() {
    return (isRed() || isBlack() || isBlue()) && LoadedObjects < 6;
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
    return seesObstacleLeft() || seesObstacleFront() || seesObstacleRight() || isYellow();
}

bool shouldAvoidBorder(int arean) {
    Area *a = AREAS[arean];
    return PX < a->xa || PY < a->ya || PX > a->xb || PY > a->yb;
}

bool shouldFollowNextDeposit() {
    return LoadedObjects >= MIN_DEP_LOADED_OBJECTS;
}

bool shouldDeposit() {
    return LoadedObjects > 0;
}

bool canDeposit() {
    return isOrange();
}

// ========== POSITION ===========

Vector *getCurrentPosition() {
    return new_vector(PX, PY);
}

Direction *getCurrentDirection() {
    return direction_fromDegrees(Compass + 90);
}

Vector *getEstimatedPosition() {
    //return lastPosition;
    return estimatedPosition;
}

Direction *getEstimatedDirection() {
    //return lastDirection;
    return estimatedDirection;
}

bool isPositionKnown() {
    Vector *position = getCurrentPosition();
    return position->x != 0 || position->y != 0;
}

void observePosition() {
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
    return (int) getEstimatedPosition()->x;
}

int estPosY() {
    return (int) getEstimatedPosition()->y;
}

int estDir() {
    return (int) direction_degrees(getEstimatedDirection());
}

// ========== SUPEROBJECTS ==========

void registerSuperobject() {
    if (SuperObj_Num == 1 && SuperObj_X != lastSuperobjectRegistered->x && SuperObj_Y != lastSuperobjectRegistered->y) {
        Vector *superobject = new_vector(SuperObj_X, SuperObj_Y);
        lastSuperobjectRegistered = superobject;
        superobjects[superobjectCount++] = superobject;
    }
}

void unregisterSuperobject(int index) {
    superobjects[index] = NULL;
    // if it's not the last, move the last to the 'index' position
    if (index != superobjectCount-1) {
        superobjects[index] = superobjects[superobjectCount-1];
    }
    superobjectCount--;
}

// ========== ACTIONS =============

void move(int left, int right) {
    WheelLeft = left;
    WheelRight = right;
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

double getSteerAngleTo(Vector *targetPoint) {
    Direction *targetDirection = vector_directionTo(getEstimatedPosition(), targetPoint);
    return getSteerAngle(direction_differenceTo(getEstimatedDirection(), targetDirection));
}

double getAngleTolerance() {
    if (isPositionKnown()) {
        return STD_ANGLE_TOLERANCE;
    }
    return 2 * STD_ANGLE_TOLERANCE;
}

void steerWithAngle(double steerAngle) {
    if (abs_double(steerAngle) < getAngleTolerance()) {
        forward(STD_SPEED);
    } else {
        double k = toDegrees(abs_double(steerAngle)) / 40;
        double motorA = STD_SPEED + k;
        double motorB = STD_SPEED - k;
        steer((int) ceil(motorA), (int) ceil(motorB), steerAngle >= 0);
    }
}

void steerTo(Vector *point) {
    double angle = getSteerAngleTo(point);
    steerWithAngle(angle);
}

void turnTo(Vector *p) {
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
    turn(steerSpeed, steerAngle
                     > 0);
}

void goTo(Vector *p, bool mayGoFaster) {
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
    Vector *point = r->points[currentRoutePoint];
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

FlowPoint *calculateNearestFlowPoint(Vector *point) {
    double distance = DBL_MAX;
    FlowPoint *nearest = NULL;
    Environment *env = getCurrentEnv();
    for (int i = 0; i < env->anchorCount; i++) {
        FlowLine *flowLine = env->flowLines[i];
        FlowPoint *current = nearestFlowPoint(flowLine, point);
        double dst = vector_distanceTo(point, current->point);
        if (distance > dst) {
            distance = dst;
            nearest = current;
        }
    }
    return nearest;
}

Vector *influenceByFlowPoint(const Vector *position, const FlowPoint *flowPoint) {
    Direction *toFlowPoint = vector_directionTo(position, flowPoint->point);
    //val force = Math.abs(/*Math.cos(toFlowPoint.difference(flowPoint.direction))*/ 1) / Math.pow(flowPoint.point.distanceTo(position) / 100, 3.0);

    /*Direction *flowDirection = toFlowPoint;
    double angleDifference = direction_difference(toFlowPoint,
                                                  vector_directionTo(position, flowPoint->point));
    if (angleDifference > M_PI / 2) {
        // we are in front of the arrow, need to mirror the pull direction
        flowDirection = direction_mirrorWith(direction_invert(flowDirection), flowPoint->direction);
    }*/

    double d = vector_distanceTo(flowPoint->point, position) / flowPoint->radius;
    double relativeAngle = direction_difference(direction_invert(flowPoint->direction), toFlowPoint);
    double weight = 1 - pow(2.0, -d * d);
    weight *= pow(cos(relativeAngle / 2), 1.0 / 2);
    //weight = 1;
    Direction *pullDirection = direction_weightedAverageWith(flowPoint->direction, toFlowPoint, weight);
    return vector_radial(pullDirection, 1.0);
}

double forceOfBorder(double distance) {
    double a = BORDER_DISTANCE;
    double b = COEFF_K;
    double x = distance;
    return max(0.0, 2 * (exp((-x) / b) - exp((-a) / b)) / (1 - exp((-a) / b)));
}

double forceOfFlowPoint(FlowPoint *flowPoint, double distance) {
    double a = flowPoint->radius;
    double b = COEFF_K;
    double x = distance;
    return max(0.0, 2 * (exp((-x) / b) - exp((-a) / b)) / (1 - exp((-a) / b)));
}

Vector *influenceByBorders(const Vector *position) {
    Vector *v = new_vector(0, 0);
    v = vector_plus(v, vector_radial(new_Direction(0.0), forceOfBorder(position->x))); // left border
    v = vector_plus(v, vector_radial(new_Direction(M_PI / 2), forceOfBorder(position->y))); // bottom border
    v = vector_plus(v, vector_radial(new_Direction(M_PI), forceOfBorder(MAP_WIDTH - position->x))); // right border
    v = vector_plus(v,
                    vector_radial(new_Direction(M_PI * 3 / 2), forceOfBorder(MAP_HEIGHT - position->y))); // top border
    return v;
}

Vector *influenceRandom(double distanceToNearestFlowPoint) {
    int offset = randn(RANDOM_VECTOR_STEP_DEVIATION * 2) - RANDOM_VECTOR_STEP_DEVIATION; // offset in degrees
    lastRandomDirection = direction_plus(lastRandomDirection, direction_fromDegrees(offset));
    double randomForce = max(0, -(1 / (distanceToNearestFlowPoint - 1) * 3) + 1);
    return vector_radial(lastRandomDirection, RANDOM_VECTOR_SIZE * randomForce);
}

Vector *influenceByDiscreteFlowPoint(const Vector *position, FlowPoint *flowPoint) {
    return vector_radial(flowPoint->direction,
                         DISCRETE_FLOWPOINT_FORCE *
                         forceOfFlowPoint(flowPoint, vector_distanceTo(position, flowPoint->point))
    );
}

Vector *influenceByDiscreteFlowPoints(const Vector *position) {
    Vector *sum = new_vector(0, 0);
    Environment *env = getCurrentEnv();
    for (int i = 0; i < env->flowPointCount; i++) {
        sum = vector_plus(sum, influenceByDiscreteFlowPoint(position, env->flowPoints[i]));
    }
    return sum;
}

Vector *calculateMoveVector(Vector *position) {
    FlowPoint *nearestFlowPoint = calculateNearestFlowPoint(position);
    Vector *infFlowPoint = influenceByFlowPoint(position, nearestFlowPoint);
    Vector *infBorders = influenceByBorders(position);
    Vector *infRandom = influenceRandom(vector_distanceTo(position, nearestFlowPoint->point));
    Vector *infDiscretes = influenceByDiscreteFlowPoints(position);
    return vector_plus(vector_plus(vector_plus(infFlowPoint, infBorders), infRandom), infDiscretes);
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

        _init_areas();
        _init_values();
        _init_flowlines();

        lastRandomDirection = new_Direction(0);

        lastPosition = estimatedPosition = getCurrentPosition();
        lastDirection = estimatedDirection = getCurrentDirection();
    }
}

int doStates() {

    // Depositing
    if (depositingTime > 0) {
        depositingTime--;
        return ACTION_DEPOSITING;
    }

    // Collecting
    if (collectingTime > 0 && (!isLongerCollecting || canCollect())) {
        collectingTime--;
        if (collectingTime < 10) isLongerCollecting = true;
        return ACTION_COLLECTING;
    }
    if (canCollect()) {
        forward(0);
        collectingTime = 38;
        LoadedObjects++;
        return ACTION_COLLECT;
    }

    isLongerCollecting = false;
    collectingTime = 0;

    // Avoid obstacle
    if (avoidingObstacleTime > 0) {
        avoidingObstacleTime--;
        return ACTION_OBSTACLE_AVOIDING;
    }
    if (shouldAvoidObstacle()) {
        avoidingObstacleTime = 6;
        if (seesObstacleLeft() /*|| isYellowLeft()*/) {
            move(-3, -1);
        } else if (seesObstacleRight() /*|| isYellowRight()*/) {
            move(-1, -3);
        } else {
            move(-2, -2);
        }
        return ACTION_OBSTACLE_AVOID;
    }

    // Avoid border
    /*if (avoidingBorderTime > 0) {
        avoidingBorderTime--;
        goTo(avoidingBorderPos);
        return 41;
    }
    if (shouldAvoidBorder(currentArea)) {
        //avoidingBorderPos = randomCoordinates(currentArea);
        avoidingBorderPos = new_vector(100, 100);
        avoidingBorderTime = AVOIDING_BORDER_TIME;
        goTo(avoidingBorderPos);
        return 40;
    }*/

    // Deposit
    if (canDeposit() && shouldDeposit()) {
        depositingTime = DEPOSITING_TIME;
        LoadedObjects = 0;
        forward(0);
        return ACTION_DEPOSIT;
    }

    // Follow route
    Route *route = getCurrentRoute();
    if (route != NULL) {
        followRoutePoint(route, true);
        return ACTION_FOLLOW_ROUTE;
    }

    // Follow flow
    Vector *position = getEstimatedPosition();
    Vector *moveVector = calculateMoveVector(position);
    Vector *target = vector_plus(position, moveVector);
    //turnTo(target);
    steerTo(target);

    return ACTION_NORMAL;
}

void Game0() {}

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
            "debug1=%d;debug2=%d;lastState=%d;estPosX=%d;estPosY=%d;estDir=%d;Duration=%d;SuperDuration=%d;bGameEnd=%d;CurAction=%d;CurGame=%d;SuperObj_Num=%d;SuperObj_X=%d;SuperObj_Y=%d;Teleport=%d;LoadedObjects=%d;US_Front=%d;US_Left=%d;US_Right=%d;CSLeft_R=%d;CSLeft_G=%d;CSLeft_B=%d;CSRight_R=%d;CSRight_G=%d;CSRight_B=%d;PositionX=%d;PositionY=%d;TM_State=%d;Compass=%d;Time=%d;WheelLeft=%d;WheelRight=%d;LED_1=%d;MyState=%d;",
            debug1, debug2, lastState, estPosX(), estPosY(), estDir(), Duration, SuperDuration, bGameEnd, CurAction,
            CurGame, SuperObj_Num, SuperObj_X,
            SuperObj_Y,
            Teleport,
            LoadedObjects, US_Front, US_Left, US_Right, CSLeft_R, CSLeft_G, CSLeft_B, CSRight_R, CSRight_G,
            CSRight_B,
            PX, PY, TM_State, Compass, Time, WheelLeft, WheelRight, LED_1, MyState);
    return info;
}

DLL_EXPORT char *GetTeamName() {
    return " ";
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
