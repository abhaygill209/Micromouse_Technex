#include <Arduino.h>
#include <TimerOne.h>

// Constant pins for different peripherals
const uint8_t trig1 = 1;
const uint8_t trig2 = 3;
const uint8_t trig3 = 5;
const uint8_t echo1 = 2;
const uint8_t echo2 = 4;
const uint8_t echo3 = 6;
const uint8_t S = 7;

const uint8_t ENA = 8;
const uint8_t IN1 = 9;
const uint8_t IN2 = 10;
const uint8_t IN3 = 11;
const uint8_t IN4 = 12;
const uint8_t ENB = 13;

// Maze dimensions
const uint8_t MAZE_SIZE = 16;
const uint8_t TARGET_X = 7;  // Center of maze
const uint8_t TARGET_Y = 7;

// Orientation and position of micromouse
volatile uint8_t orientation = 1;  // 0=East, 1=North, 2=West, 3=South
volatile uint8_t x = 0, y = 0;     // Starting position

// Wall and flood-fill arrays
uint8_t wall[MAZE_SIZE][MAZE_SIZE];
uint8_t grid[MAZE_SIZE][MAZE_SIZE];
bool visited[MAZE_SIZE][MAZE_SIZE];

// Parameters
const uint8_t WALL_DIST = 5;  // Distance to be considered as a wall
const uint8_t MOTOR_SPEED = 150;
const uint16_t TURN_DELAY = 500;
const uint16_t MOVE_DELAY = 800;

// Direction constants
const uint8_t NORTH = 0b0001;
const uint8_t EAST  = 0b0010;
const uint8_t SOUTH = 0b0100;
const uint8_t WEST  = 0b1000;

// Timer interrupt variables
volatile bool sensorReadFlag = false;
volatile bool navigationFlag = false;
volatile bool motorUpdateFlag = false;
volatile uint16_t timerCounter = 0;

// Motor control variables
volatile bool isMoving = false;
volatile uint8_t currentMove = 0;  // 0=stop, 1=forward, 2=left, 3=right, 4=back
volatile uint16_t moveTimer = 0;

// Sensor data
volatile float distanceL = 0;
volatile float distanceS = 0;
volatile float distanceR = 0;

// Timer ISR - runs every 10ms
void timerISR() {
    timerCounter++;
    
    // Sensor reading every 50ms (5 * 10ms)
    if (timerCounter % 5 == 0) {
        sensorReadFlag = true;
    }
    
    // Navigation update every 100ms (10 * 10ms)
    if (timerCounter % 10 == 0) {
        navigationFlag = true;
    }
    
    // Motor update every 10ms
    motorUpdateFlag = true;
    
    // Handle motor timing
    if (isMoving && moveTimer > 0) {
        moveTimer--;
        if (moveTimer == 0) {
            isMoving = false;
            currentMove = 0;  // Stop
        }
    }
}

// flood fill 
void floodfill_recursive(uint8_t coord_x, uint8_t coord_y, uint8_t depth = 0) {
    // Stack overflow protection
    if (depth > MAZE_SIZE * MAZE_SIZE) {
        Serial.println("Stack overflow protection triggered!");
        return;
    }
    
    // Bounds checking
    if (coord_x >= MAZE_SIZE || coord_y >= MAZE_SIZE) {
        return;
    }
    
    // Calculate minimum value from accessible neighbors
    uint8_t min_val = mini(coord_x, coord_y);
    
    // If current cell value is already optimal, no need to continue
    if (grid[coord_x][coord_y] <= min_val + 1) {
        return;
    }
    
    // Update current cell
    grid[coord_x][coord_y] = min_val + 1;
    
    // Recursively update accessible neighbors
    // Check South
    if (!(wall[coord_x][coord_y] & SOUTH) && coord_y > 0) {
        floodfill_recursive(coord_x, coord_y - 1, depth + 1);
    }
    
    // Check East
    if (!(wall[coord_x][coord_y] & EAST) && coord_x < MAZE_SIZE - 1) {
        floodfill_recursive(coord_x + 1, coord_y, depth + 1);
    }
    
    // Check North
    if (!(wall[coord_x][coord_y] & NORTH) && coord_y < MAZE_SIZE - 1) {
        floodfill_recursive(coord_x, coord_y + 1, depth + 1);
    }
    
    // Check West
    if (!(wall[coord_x][coord_y] & WEST) && coord_x > 0) {
        floodfill_recursive(coord_x - 1, coord_y, depth + 1);
    }
}


float distance(uint8_t trig, uint8_t echo) {
    digitalWrite(trig, LOW);
    delayMicroseconds(2);
    digitalWrite(trig, HIGH);
    delayMicroseconds(10);
    digitalWrite(trig, LOW);
    
    unsigned long duration = pulseIn(echo, HIGH, 30000);  // 30ms timeout
    
    float dist = (duration * 0.0343) / 2;
    return dist;
}

bool if_wall(float dist) {
    return (dist < WALL_DIST && dist > 0);
}

void wallupdate(uint8_t NSEW) {
    if (x < MAZE_SIZE && y < MAZE_SIZE) {
        wall[x][y] |= NSEW;
    }
}

void setMotorCommand(uint8_t command, uint16_t duration_ms) {
    currentMove = command;
    moveTimer = duration_ms / 10;  // Convert to 10ms ticks
    isMoving = true;
}

void updateMotors() {
    switch(currentMove) {
        case 0:  // Stop
            digitalWrite(IN1, LOW);
            digitalWrite(IN2, LOW);
            digitalWrite(IN3, LOW);
            digitalWrite(IN4, LOW);
            analogWrite(ENA, 0);
            analogWrite(ENB, 0);
            break;
            
        case 1:  // Forward
            digitalWrite(IN1, HIGH);
            digitalWrite(IN2, LOW);
            digitalWrite(IN3, HIGH);
            digitalWrite(IN4, LOW);
            analogWrite(ENA, MOTOR_SPEED);
            analogWrite(ENB, MOTOR_SPEED);
            break;
            
        case 2:  // Turn Left
            digitalWrite(IN1, LOW);
            digitalWrite(IN2, HIGH);
            digitalWrite(IN3, HIGH);
            digitalWrite(IN4, LOW);
            analogWrite(ENA, MOTOR_SPEED);
            analogWrite(ENB, MOTOR_SPEED);
            break;
            
        case 3:  // Turn Right
            digitalWrite(IN1, HIGH);
            digitalWrite(IN2, LOW);
            digitalWrite(IN3, LOW);
            digitalWrite(IN4, HIGH);
            analogWrite(ENA, MOTOR_SPEED);
            analogWrite(ENB, MOTOR_SPEED);
            break;
            
        case 4:  // Backward (for debugging)
            digitalWrite(IN1, LOW);
            digitalWrite(IN2, HIGH);
            digitalWrite(IN3, LOW);
            digitalWrite(IN4, HIGH);
            analogWrite(ENA, MOTOR_SPEED);
            analogWrite(ENB, MOTOR_SPEED);
            break;
    }
}

void strait() {
    if (!isMoving) {
        setMotorCommand(1, MOVE_DELAY);  // Move forward
        // Update position will be done when movement completes
    }
}

void left() {
    if (!isMoving) {
        setMotorCommand(2, TURN_DELAY);  // Turn left
        orientation = (orientation + 3) % 4;
    }
}

void right() {
    if (!isMoving) {
        setMotorCommand(3, TURN_DELAY);  // Turn right
        orientation = (orientation + 1) % 4;
    }
}

void back() {
    if (!isMoving) {
        // Turn around (two right turns)
        setMotorCommand(3, TURN_DELAY * 2);
        orientation = (orientation + 2) % 4;
    }
}

void updatePosition() {
    // Update position based on orientation after forward movement
    switch(orientation) {
        case 0: if (x < MAZE_SIZE-1) x++; break;  // East
        case 1: if (y < MAZE_SIZE-1) y++; break;  // North
        case 2: if (x > 0) x--; break;            // West
        case 3: if (y > 0) y--; break;            // South
    }
}

// Initialize flood-fill grid
void initializeGrid() {
    // Clear arrays
    memset(wall, 0, sizeof(wall));
    memset(visited, false, sizeof(visited));
    
    // Initialize grid with Manhattan distance to center
    for (uint8_t i = 0; i < MAZE_SIZE; i++) {
        for (uint8_t j = 0; j < MAZE_SIZE; j++) {
            grid[i][j] = abs(i - TARGET_X) + abs(j - TARGET_Y);
        }
    }
    
    // Add boundary walls
    for (uint8_t i = 0; i < MAZE_SIZE; i++) {
        wall[0][i] |= WEST;              // Left boundary
        wall[MAZE_SIZE-1][i] |= EAST;    // Right boundary
        wall[i][0] |= SOUTH;             // Bottom boundary
        wall[i][MAZE_SIZE-1] |= NORTH;   // Top boundary
    }
}

// Fixed mini function with bounds checking
uint8_t mini(uint8_t coord_x, uint8_t coord_y) {
    uint8_t min_val = 255;
    
    // Check South
    if (!(wall[coord_x][coord_y] & SOUTH) && coord_y > 0) {
        min_val = min(min_val, grid[coord_x][coord_y - 1]);
    }
    // Check East
    if (!(wall[coord_x][coord_y] & EAST) && coord_x < MAZE_SIZE-1) {
        min_val = min(min_val, grid[coord_x + 1][coord_y]);
    }
    // Check North
    if (!(wall[coord_x][coord_y] & NORTH) && coord_y < MAZE_SIZE-1) {
        min_val = min(min_val, grid[coord_x][coord_y + 1]);
    }
    // Check West
    if (!(wall[coord_x][coord_y] & WEST) && coord_x > 0) {
        min_val = min(min_val, grid[coord_x - 1][coord_y]);
    }
    
    return min_val;
}

// Improved direction function with bounds checking
uint8_t direction() {
    uint8_t val = grid[x][y];
    uint8_t best_dir = 255;
    uint8_t min_val = 255;
    
    // Check all four directions and find the one with minimum value
    if (x > 0 && !(wall[x][y] & WEST) && grid[x-1][y] < min_val) {
        min_val = grid[x-1][y];
        best_dir = 2;  // West
    }
    if (y < MAZE_SIZE-1 && !(wall[x][y] & NORTH) && grid[x][y+1] < min_val) {
        min_val = grid[x][y+1];
        best_dir = 1;  // North
    }
    if (x < MAZE_SIZE-1 && !(wall[x][y] & EAST) && grid[x+1][y] < min_val) {
        min_val = grid[x+1][y];
        best_dir = 0;  // East
    }
    if (y > 0 && !(wall[x][y] & SOUTH) && grid[x][y-1] < min_val) {
        min_val = grid[x][y-1];
        best_dir = 3;  // South
    }
    
    return best_dir;
}

void readSensors() {
    distanceL = distance(trig1, echo1);
    distanceS = distance(trig2, echo2);
    distanceR = distance(trig3, echo3);
}

void processNavigation() {
    // Check if reached target
    if (x == TARGET_X && y == TARGET_Y) {
        robotState = TARGET_REACHED;
        return;
    }
    
    // Detect walls
    bool wall_L = if_wall(distanceL);
    bool wall_S = if_wall(distanceS);
    bool wall_R = if_wall(distanceR);
    
    // Map walls based on current orientation
    uint8_t NSEW = 0;
    switch(orientation) {
        case 0:  // Facing East
            NSEW = (wall_L ? NORTH : 0) | (wall_S ? EAST : 0) | (wall_R ? SOUTH : 0);
            break;
        case 1:  // Facing North
            NSEW = (wall_L ? WEST : 0) | (wall_S ? NORTH : 0) | (wall_R ? EAST : 0);
            break;
        case 2:  // Facing West
            NSEW = (wall_L ? SOUTH : 0) | (wall_S ? WEST : 0) | (wall_R ? NORTH : 0);
            break;
        case 3:  // Facing South
            NSEW = (wall_L ? EAST : 0) | (wall_S ? SOUTH : 0) | (wall_R ? WEST : 0);
            break;
    }
    
    // Update wall information
    wallupdate(NSEW);
    
    // Run flood-fill algorithm
    floodfill(x, y);
    
    // Determine next move if not currently moving
    if (!isMoving) {
        uint8_t dir = direction();
        if (dir == 255) {
            Serial.println("No valid direction found!");
            robotState = IDLE;
            return;
        }
        
        // Convert global direction to relative movement
        uint8_t relative_dir = (dir - orientation + 4) % 4;
        
        switch(relative_dir) {
            case 0: 
                strait(); 
                robotState = MOVING;
                break;  // Forward
            case 1: 
                right(); 
                robotState = MOVING;
                break;   // Right
            case 2: 
                back(); 
                robotState = MOVING;
                break;    // Back
            case 3: 
                left(); 
                robotState = MOVING;
                break;    // Left
        }
    }
}

void setup() {
    Serial.begin(9600);
    
    // Initialize maze data structures
    initializeGrid();
    
    // Sensors
    pinMode(trig1, OUTPUT);
    pinMode(trig2, OUTPUT);
    pinMode(trig3, OUTPUT);
    pinMode(echo1, INPUT);
    pinMode(echo2, INPUT);
    pinMode(echo3, INPUT);
    
    // Motors
    pinMode(ENA, OUTPUT);
    pinMode(ENB, OUTPUT);
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
    
    // Initialize timer interrupt (10ms period)
    Timer1.initialize(10000);  // 10ms = 10,000 microseconds
    Timer1.attachInterrupt(timerISR);
    
    Serial.println("Micromouse initialized with timer interrupts");
    delay(2000);  // Give time to place robot
    
    robotState = READING_SENSORS;
}


void loop() {
    // Handle sensor reading
    if (sensorReadFlag) {
        sensorReadFlag = false;
        if (robotState == READING_SENSORS || robotState == PROCESSING_MAZE) {
            readSensors();
            robotState = PROCESSING_MAZE;
        }
    }
    
    // Handle navigation processing
    if (navigationFlag) {
        navigationFlag = false;
        if (robotState == PROCESSING_MAZE) {
            processNavigation();
        }
    }
    
    // Handle motor updates
    if (motorUpdateFlag) {
        motorUpdateFlag = false;
        updateMotors();
        
        // Check if movement completed
        if (robotState == MOVING && !isMoving) {
            // Update position if it was a forward movement
            if (currentMove == 1) {  // Was moving forward
                updatePosition();
            }
            robotState = READING_SENSORS;
        }
    }
    
    // Handle target reached state
    if (robotState == TARGET_REACHED) {
        Serial.println("Target reached!");
        setMotorCommand(0, 0);  // Stop motors
        Timer1.detachInterrupt();  // Stop timer
        while(1) {
            // Celebration or final state
            delay(1000);
            Serial.println("Mission accomplished!");
        }
    }
    
    // Debug output (non-blocking)
    static unsigned long lastDebugTime = 0;
    if (millis() - lastDebugTime > 500) {  // Every 500ms
        lastDebugTime = millis();
        Serial.print("State: ");
        Serial.print(robotState);
        Serial.print(" Pos: (");
        Serial.print(x);
        Serial.print(",");
        Serial.print(y);
        Serial.print(") Orient: ");
        Serial.print(orientation);
        Serial.print(" Moving: ");
        Serial.print(isMoving);
        Serial.print(" Sensors: L=");
        Serial.print(distanceL);
        Serial.print(" S=");
        Serial.print(distanceS);
        Serial.print(" R=");
        Serial.println(distanceR);
    }
}
