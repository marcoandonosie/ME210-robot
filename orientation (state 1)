/*----------------Helper Functions--------------------------*/
//use these later
bool isLocalMinimum(float current_distance) {
    static float last_distance = 9999;
    bool isMin = current_distance < last_distance;
    last_distance = current_distance;
    return isMin;
}

int findSmallest(float arr[]) {
    int minIndex = 0;
    for (int i = 1; i < 4; i++) {
        if (arr[i] < arr[minIndex]) {
            minIndex = i;
        }
    }
    return minIndex;
}

int findSecondSmallest(float arr[], int minIndex) {
    int secondMinIndex = (minIndex == 0) ? 1 : 0;
    for (int i = 0; i < 4; i++) {
        if (i != minIndex && arr[i] < arr[secondMinIndex]) {
            secondMinIndex = i;
        }
    }
    return secondMinIndex;
}

void startRotation(int direction) {
    if (direction == CLOCKWISE) {
        // Set motors for clockwise rotation
    } else {
        // Set motors for counterclockwise rotation
    }
}

void stopRotation() {
    // Stop motors
}

void rotate(int direction, unsigned long duration) {
    startRotation(direction);
    delay(duration);
    stopRotation();
}

float readUltrasonicSensor() {
    // Return ultrasonic sensor reading
    return analogRead(PIN_SIGNAL_IN);
}



/*----------------Actual Functions--------------------------*/
//currently just pseudocode

void handleOrientationState() {
    static bool rotation_started = false;
    static bool found_minima[4] = {false, false, false, false};
    static unsigned long t_minima[4] = {0, 0, 0, 0};
    static float d_minima[4] = {9999, 9999, 9999, 9999}; // Initialize with large values

    if (!rotation_started) {
        t_start = millis();
        startRotation(CLOCKWISE);
        rotation_started = true;
    }

    float current_distance = readUltrasonicSensor();

    // Detect local minima
    if (isLocalMinimum(current_distance)) {
        for (int i = 0; i < 4; i++) {
            if (!found_minima[i]) {
                t_minima[i] = millis();
                d_minima[i] = current_distance;
                found_minima[i] = true;
                break;
            }
        }
    }

    // Once 4 minima are found
    if (found_minima[3]) {
        stopRotation();
        
        // Identify the two smallest minima (likely west & south)
        int min1 = findSmallest(d_minima);
        int min2 = findSecondSmallest(d_minima, min1);
        
        unsigned long tWest = t_minima[min1];
        unsigned long tSouth = t_minima[min2];

        // Compute rotation times
        unsigned long time_to_west = millis() - tWest;
        unsigned long time_to_90deg = tWest - tSouth;

        // Align the robot
        rotate(COUNTERCLOCKWISE, time_to_west);
        rotate(CLOCKWISE, time_to_90deg);

        // Transition to the next state
        state = STATE_MOVE_FORWARD;
    }
}
