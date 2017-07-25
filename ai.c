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
#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include <stdlib.h>
#include <float.h>

#define DLL_EXPORT extern __declspec(dllexport)
#define false 0
#define true 1

#endif

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

double vector_distanceTo(Vector *A, const Vector *B) {
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

int MIN_DEP_LOADED_OBJECTS = 4;
int STD_SPEED = 2;
double STD_ANGLE_TOLERANCE = 4 * M_PI / 180;
int AVOIDING_BORDER_TIME = 20;
int DEPOSITING_TIME = 40;
int RANDOM_COORDINATES_PADDING = 30;

//========== PROGRAM variables ==========
bool initialized = false;
int ticks = 0;

//========== STATE variables ==========
int collectingTime = 0;
bool isLongerCollecting = false;
int currentArea = 0;
int currentCheckpoint = 0;
int avoidingBorderTime = 0;
Vector *avoidingBorderPos;
int depositingTime = 0;

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

#define ANCHORS_COUNT 7
Anchor *ANCHORS[ANCHORS_COUNT];
int _index_anchor = 0;

void _anchor(int x, int y, int radius) {
    ANCHORS[_index_anchor++] = new_Anchor(new_vector(x, y), radius);
}

void _init_anchors() {
    //####################ANCHOR ####################
    _anchor(102, 44, 40);
    _anchor(93, 58, 40);
    _anchor(110, 190, 40);
    _anchor(155, 210, 40);
    _anchor(175, 195, 40);
    _anchor(205, 113, 40);
    _anchor(160, 40, 40);
}

FlowLine *FLOWLINES[ANCHORS_COUNT];

void _init_flowlines() {
    for (int i = 0; i < ANCHORS_COUNT; i++) {
        Anchor *aa = ANCHORS[i];
        Anchor *ab = ANCHORS[(i + 1) % ANCHORS_COUNT];
        FLOWLINES[i] = new_FlowLine(aa, ab);
    }
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
    o->s1 = 1;
    o->s2 = 7;
    o->s3 = 15;
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

int angleDiffTo(Vector *p) {
    return angleDiff(Compass, angleTo(p));
}

double toRange(double value, double min, double max) {
    return min(max(min, value), max);
}

double abs_double(double value) {
    return value >= 0 ? value : -value;
}

// =========== COLORS ============

bool isYellowRight() { return CSRight_R > 200 && CSRight_G > 200 && CSRight_B < 50; }

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

bool isOrange() { return isOrangeRight() || isOrangeLeft(); }

// =========== AREAS ==========

Vector *randomCoordinates(int arean) {
    Area *a = AREAS[arean];
    int x = randn(a->xb - a->xa - 2 * RANDOM_COORDINATES_PADDING) + a->xa + RANDOM_COORDINATES_PADDING;
    int y = randn(a->yb - a->ya - 2 * RANDOM_COORDINATES_PADDING) + a->ya + RANDOM_COORDINATES_PADDING;
    return new_vector(x, y);
}

// =========== CHECKS ==========

bool canCollect() {
    return isRed() || isGreen() || isBlack();
}

bool shouldAvoidObstacle() {
    return US_Right < 6 || US_Front < 6 || US_Left < 6 || isYellow();
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

Vector *getEstimatedPosition() {
    return new_vector(PX, PY);
}

Direction *getCurrentDirection() {
    return direction_fromDegrees(Compass + 90);
}

Direction *getEstimatedDirection() {
    return getCurrentDirection();
}

// ========== ACTIONS =============

void move(int left, int right) {
    WheelLeft = left;
    WheelRight = right;
}

void forward(int speed) {
    move(speed, speed);
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

void steerWithAngle(double steerAngle) {
    if (abs_double(steerAngle) < STD_ANGLE_TOLERANCE) {
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
    if (steerAngle < 30) {
        steerSpeed = 1;
    } else if (steerAngle < 60) {
        steerSpeed = 2;
    } else {
        steerSpeed = 4;
    }
    turn(steerSpeed, steerAngle > 0);
}

void goTo(Vector *p) {
    double steerAngle = getSteerAngleTo(p);
    if (abs_double(steerAngle) > STD_ANGLE_TOLERANCE) {
        turnTo(p);
    } else {
        forward(STD_SPEED);
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
    for (int i = 0; i < ANCHORS_COUNT; i++) {
        FlowLine *flowLine = FLOWLINES[i];
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

    Direction *flowDirection = toFlowPoint;
    double angleDifference = direction_difference(toFlowPoint,
                                                  vector_directionTo(position, flowPoint->point));
    if (angleDifference > M_PI / 2) {
        // we are in front of the arrow, need to mirror the pull direction
        flowDirection = direction_mirrorWith(direction_invert(flowDirection), flowPoint->direction);
    }

    double d = vector_distanceTo(flowPoint->point, position) / flowPoint->radius;
    double relativeAngle = direction_difference(direction_invert(flowPoint->direction), toFlowPoint);
    double weight = 1 - pow(2.0, -d * d);
    weight *= pow(cos(relativeAngle / 2), 1.0 / 2);
    //weight = 1;
    Direction *pullDirection = direction_weightedAverageWith(flowPoint->direction, toFlowPoint, weight);
    return vector_radial(pullDirection, 1.0);
}

Vector *calculateMoveVector(Vector *position) {
    FlowPoint *nearestFlowPoint = calculateNearestFlowPoint(position);
    Vector *influence = influenceByFlowPoint(position, nearestFlowPoint);
    return influence;
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
        _init_anchors();
        _init_flowlines();
    }
}

int doStates() {

    // Depositing
    if (depositingTime > 0) {
        depositingTime--;
        return 51;
    }

    // Collecting
    if (collectingTime > 0 && (!isLongerCollecting || canCollect())) {
        collectingTime--;
        if (collectingTime < 10) isLongerCollecting = true;
        return 21;
    }
    if (canCollect()) {
        forward(0);
        collectingTime = 38;
        LoadedObjects++;
        return 20;
    }

    collectingTime = 0;

    // Avoid obstacle
    if (shouldAvoidObstacle()) {
        move(-STD_SPEED - 1, STD_SPEED);
        return 30;
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
        return 50;
    }

    // Follow flow
    Vector *position = getEstimatedPosition();
    Vector *moveVector = calculateMoveVector(position);
    Vector *target = vector_plus(position, moveVector);
    //turnTo(target);
    steerTo(target);

    debug1 = (int) toDegrees(*getEstimatedDirection());
    debug2 = (int) toDegrees(*vector_directionTo(position, target));
    //move(0,0);
    //move(1,1);
    return 1;
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

    lastState = doStates();

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
            "debug1=%d;debug2=%d;Duration=%d;SuperDuration=%d;bGameEnd=%d;CurAction=%d;CurGame=%d;SuperObj_Num=%d;SuperObj_X=%d;SuperObj_Y=%d;Teleport=%d;LoadedObjects=%d;US_Front=%d;US_Left=%d;US_Right=%d;CSLeft_R=%d;CSLeft_G=%d;CSLeft_B=%d;CSRight_R=%d;CSRight_G=%d;CSRight_B=%d;PositionX=%d;PositionY=%d;TM_State=%d;Compass=%d;Time=%d;WheelLeft=%d;WheelRight=%d;LED_1=%d;MyState=%d;",
            debug1, debug2, Duration, SuperDuration, bGameEnd, CurAction, CurGame, SuperObj_Num, SuperObj_X, SuperObj_Y,
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
