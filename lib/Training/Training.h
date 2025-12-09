#ifndef TRAINING_H
#define TRAINING_H

#include <Arduino.h>

class Training {
public:
    Training();
    void begin();
    
    // Training methods - to be implemented
    void startTraining();
    void stopTraining();
    bool isTraining();
    
    // Learning methods - to be implemented
    void executeLearnedBehavior();
    bool hasLearnedBehavior();
    
    // Data management - to be implemented
    void saveModel();
    void loadModel();
    void resetModel();

private:
    bool trainingActive;
    bool modelLoaded;
};

#endif // TRAINING_H
