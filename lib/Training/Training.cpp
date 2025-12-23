#include "Training.h"
#include <EEPROM.h>

// ---------------- CONFIG ----------------
#define NUM_STATES   3
#define NUM_ACTIONS  3

#define EEPROM_SIZE 512
#define Q_ADDR 0

// Learning parameters
static float alpha   = 0.3f;
static float gamma   = 0.9f;
static float epsilon = 0.4f;
static float epsilonMin = 0.0f;
static float epsilonDecay = 0.995f;

// Q-table
static float Q[NUM_STATES][NUM_ACTIONS];

// --------------------------------------

Training::Training() : trainingActive(true), modelLoaded(false) {}

void Training::begin() {
    for (int s = 0; s < NUM_STATES; s++)
        for (int a = 0; a < NUM_ACTIONS; a++)
            Q[s][a] = 0.0f;

    Serial.println("Training initialized (Q-learning)");
}

uint8_t Training::selectAction(uint8_t state) {
    // EXECUTION MODE → deterministic
    if (!trainingActive) {
        uint8_t bestA = 0;
        float bestQ = Q[state][0];

        for (int a = 1; a < NUM_ACTIONS; a++) {
            if (Q[state][a] > bestQ) {
                bestQ = Q[state][a];
                bestA = a;
            }
        }
        return bestA;
    }

    // TRAINING MODE → epsilon-greedy
    if (((float)random(1000) / 1000.0f) < epsilon) {
        return random(NUM_ACTIONS);
    }

    uint8_t bestA = 0;
    float bestQ = Q[state][0];
    for (int a = 1; a < NUM_ACTIONS; a++) {
        if (Q[state][a] > bestQ) {
            bestQ = Q[state][a];
            bestA = a;
        }
    }
    return bestA;
}

void Training::updateQ(uint8_t state, uint8_t action, float reward, uint8_t nextState) {
    if (!trainingActive) return;

    float maxNextQ = Q[nextState][0];
    for (int a = 1; a < NUM_ACTIONS; a++)
        if (Q[nextState][a] > maxNextQ)
            maxNextQ = Q[nextState][a];

    Q[state][action] += alpha *
        (reward + gamma * maxNextQ - Q[state][action]);

    if (epsilon > epsilonMin)
        epsilon *= epsilonDecay;
}

void Training::startTraining() {
    trainingActive = true;
}

void Training::stopTraining() {
    trainingActive = false;
    epsilon = 0.0f;
    modelLoaded = true;
}

bool Training::isTraining() {
    return trainingActive;
}

bool Training::hasLearnedBehavior() {
    return modelLoaded;
}

float Training::getEpsilon() {
    return epsilon;
}

// ---------- Persistence ----------
void Training::saveModel() {
    EEPROM.begin(EEPROM_SIZE);
    int addr = Q_ADDR;

    for (int s = 0; s < NUM_STATES; s++)
        for (int a = 0; a < NUM_ACTIONS; a++) {
            EEPROM.put(addr, Q[s][a]);
            addr += sizeof(float);
        }

    EEPROM.commit();
    EEPROM.end();
}

void Training::loadModel() {
    EEPROM.begin(EEPROM_SIZE);
    int addr = Q_ADDR;

    for (int s = 0; s < NUM_STATES; s++)
        for (int a = 0; a < NUM_ACTIONS; a++) {
            EEPROM.get(addr, Q[s][a]);
            addr += sizeof(float);
        }

    EEPROM.end();
    trainingActive = false;
    epsilon = 0.0f;
    modelLoaded = true;
}

void Training::resetModel() {
    for (int s = 0; s < NUM_STATES; s++)
        for (int a = 0; a < NUM_ACTIONS; a++)
            Q[s][a] = 0.0f;

    epsilon = 0.4f;
    trainingActive = true;
    modelLoaded = false;
}
