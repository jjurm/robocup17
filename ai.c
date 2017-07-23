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
#pragma ide diagnostic ignored "OCUnusedGlobalDeclarationInspection"
#define CsBot_AI_H//DO NOT delete this line
#ifndef CSBOT_REAL

#include <windows.h>
#include <stdio.h>
#include <math.h>

#define DLL_EXPORT extern __declspec(dllexport)
#define false 0
#define true 1
#endif

#include <stdint.h>
#include <stdlib.h>

#define CsBot_AI_C //DO NOT delete this line

typedef int bool;

//The robot ID : It must be two char, such as '00','kl' or 'Cr'.
char AI_MyID[2] = {'0', '2'};

typedef struct {
    int x, y;
} pos;

pos *new_pos(int xa, int ya) {
    pos *p = malloc(sizeof(pos));
    p->x = xa;
    p->y = ya;
    return p;
}

typedef struct {
    int xa, xb, ya, yb;
} area;

area *new_area(int xa, int xb, int ya, int yb) {
    area *o = malloc(sizeof(area));
    o->xa = xa;
    o->xb = xb;
    o->ya = ya;
    o->yb = yb;
    return o;
}

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

// constants
#define BORDER_MARGIN 20
area AREAS[] = {
        {0 + BORDER_MARGIN, 360 - BORDER_MARGIN, BORDER_MARGIN, 270 - BORDER_MARGIN}
};
pos CHECKPOINTS[] = {
        {1, 2},
        {3, 4}
};
int MIN_DEP_LOADED_OBJECTS = 4;
int STD_SPEED = 2;
int STD_ANGLE_TOLERANCE = 45;
int AVOIDING_BORDER_TIME = 20;
int DEPOSITING_TIME = 40;
int RANDOM_COORDINATES_PADDING = 30;

// program variables
bool initialized = false;

// state variables
bool isCollecting = false;
int currentArea = 0;
int currentCheckpoint = 0;
int avoidingBorderTime = 0;
pos *avoidingBorderPos;
int depositingTime = 0;

// temporary variables
int lastState = 0;


// =========== PROGRAM ============

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

int angleTo(pos *p) {
    int a = (int) (atan2(p->x - PX, p->y - PY) * 180 / 3.141592658);
    if (a < 0) a += 360;
    return a;
}

int angleDiff(int a, int b) {
    int d1 = b - a;
    if (d1 > 180) d1 -= 360;
    if (d1 < -180) d1 += 360;
    return d1;
}

int angleDiffTo(pos *p) {
    return angleDiff(Compass, angleTo(p));
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

pos *randomCoordinates(int arean) {
    area *a = &AREAS[arean];
    int x = randn(a->xb - a->xa - 2 * RANDOM_COORDINATES_PADDING) + a->xa + RANDOM_COORDINATES_PADDING;
    int y = randn(a->yb - a->ya - 2 * RANDOM_COORDINATES_PADDING) + a->ya + RANDOM_COORDINATES_PADDING;
    return new_pos(x, y);
}

// =========== CHECKS ==========

bool canCollect() {
    return isRed() || isGreen() || isBlack();
}

bool shouldAvoidObstacle() {
    return US_Right < 10 || US_Front < 10 || US_Left < 10 || isYellow();
}

bool shouldAvoidBorder(int arean) {
    area *a = &AREAS[arean];
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

void turnTo(pos *p) {
    int ad = angleDiffTo(p);
    turn(STD_SPEED, ad > 0);
}

void goTo(pos *p) {
    int ad = angleDiffTo(p);
    if (abs(ad) > STD_ANGLE_TOLERANCE) {
        turnTo(p);
    } else {
        forward(STD_SPEED);
    }
}

// ========== PROGRAM ========

void init() {
    if (!initialized) {
        initialized = true;

        rnd_state_o = *new_rnd_state();
    }
}

int doStates() {

    // Depositing
    if (depositingTime > 0) {
        depositingTime--;
        return 51;
    }

    // Collecting
    if (isCollecting || canCollect()) {
        return 21;
    }
    if (canCollect()) {
        isCollecting = true;
        LoadedObjects++;
        return 20;
    }

    isCollecting = false;

    // Avoid obstacle
    if (shouldAvoidObstacle()) {
        move(-STD_SPEED - 1, STD_SPEED);
        return 30;
    }

    // Avoid border
    if (avoidingBorderTime > 0) {
        avoidingBorderTime--;
        goTo(avoidingBorderPos);
        return 41;
    }
    if (shouldAvoidBorder(currentArea)) {
        //avoidingBorderPos = randomCoordinates(currentArea);
        avoidingBorderPos = new_pos(100, 100);
        avoidingBorderTime = AVOIDING_BORDER_TIME;
        goTo(avoidingBorderPos);
        return 40;
    }

    // Deposit
    if (canDeposit() && shouldDeposit()) {
        depositingTime = DEPOSITING_TIME;
        LoadedObjects = 0;
        return 50;
    }

    // Normal walk
    if (true) {
        move(STD_SPEED, STD_SPEED);
    }
}

void Game0() {}

void Game1() {

    if (SuperDuration > 0) {
        SuperDuration--;
    } else if (Duration > 0) {
        Duration--;
    }

    lastState = doStates();

    if (isCollecting || depositingTime > 0) {
        LED_1 = 2;
    } else {
        LED_1 = 0;
    }

}

// ========== COSPACE Stuff (do not edit)

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
            "Duration=%d;SuperDuration=%d;bGameEnd=%d;CurAction=%d;CurGame=%d;SuperObj_Num=%d;SuperObj_X=%d;SuperObj_Y=%d;Teleport=%d;LoadedObjects=%d;US_Front=%d;US_Left=%d;US_Right=%d;CSLeft_R=%d;CSLeft_G=%d;CSLeft_B=%d;CSRight_R=%d;CSRight_G=%d;CSRight_B=%d;PositionX=%d;PositionY=%d;TM_State=%d;Compass=%d;Time=%d;WheelLeft=%d;WheelRight=%d;LED_1=%d;MyState=%d;",
            Duration, SuperDuration, bGameEnd, CurAction, CurGame, SuperObj_Num, SuperObj_X, SuperObj_Y, Teleport,
            LoadedObjects, US_Front, US_Left, US_Right, CSLeft_R, CSLeft_G, CSLeft_B, CSRight_R, CSRight_G, CSRight_B,
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

DLL_EXPORT void SetDataAI(volatile int *packet, volatile int *AI_IN) {

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