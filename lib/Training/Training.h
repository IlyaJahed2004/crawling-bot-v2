#ifndef TRAINING_H
#define TRAINING_H

#include <Arduino.h>

class Training {
public:
    Training();
    void begin();

    // RL core
    uint8_t selectAction(uint8_t state);
    void updateQ(uint8_t state, uint8_t action, float reward, uint8_t nextState);

    // Training control
    void startTraining();
    void stopTraining();
    bool isTraining();

    // Execution
    bool hasLearnedBehavior();
    float getEpsilon();

    // Persistence (safe, optional)
    void saveModel();
    void loadModel();
    void resetModel();

private:
    bool trainingActive;
    bool modelLoaded;
};

#endif
