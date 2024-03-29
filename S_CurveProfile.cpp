#include <iostream>
#include <stdio.h>
#include <math.h>
#include <stdexcept>
#include <vector>

struct Constraints {
    float maxVelocity;
    float maxAcceleration;
    float maxDeceleration;
    float maxJerk;
};

struct QuadraticResult {
    float first;
    float second;
};

QuadraticResult quadraticFormula(float a, float b, float c) {
    QuadraticResult result;
    float discriminant = b * b - 4 * a * c;

    if (discriminant > 0) {
        result.first = (-b + sqrt(discriminant)) / (2 * a);
        result.second = (-b - sqrt(discriminant)) / (2 * a);
    } else if (discriminant == 0) {
        result.first = -b / (2 * a);
        result.second = result.first; // Both roots are the same for a zero discriminant
    } else {
        // Complex roots, set them to NaN (Not a Number)
        result.first = std::numeric_limits<float>::quiet_NaN();
        result.second = std::numeric_limits<float>::quiet_NaN();
    }

    return result;
}

class S_CurveProfile {
    private:
        Constraints constraints;

        bool  fullAcceleration;
        bool  isReversed;
        float minDistance;
        float fullDistance;

        float distance;

        float timePhase[7];
        float distPhase[7];
        float velPhase[7];
        float accPhase[7];
        float jerkPhase[7];

        void setJerkPhase() {
            this->jerkPhase[0] = this->constraints.maxJerk;
            this->jerkPhase[1] = 0;
            this->jerkPhase[2] = -this->constraints.maxJerk;
            this->jerkPhase[3] = 0;
            this->jerkPhase[4] = -this->constraints.maxJerk;
            this->jerkPhase[5] = 0;
            this->jerkPhase[6] = this->constraints.maxJerk;
        }

        void setTimePhase(int stages) {
            if (stages == 4) {
                this->timePhase[1] = this->timePhase[3] = this->timePhase[5] = 0;
                this->timePhase[0] = this->timePhase[2] = this->timePhase[4] = this->timePhase[6] = cbrt(this->distance / this->constraints.maxJerk / 2);
            }
            else if (stages == 5) {
                this->timePhase[0] = this->timePhase[2] = this->timePhase[4] = this->timePhase[6] = sqrt(this->constraints.maxVelocity / this->constraints.maxJerk);
                this->timePhase[1] = this->timePhase[5] = 0;
                this->timePhase[3] = (this->distance - this->constraints.maxVelocity * this->timePhase[0] * 2) / this->constraints.maxVelocity;
            }
            else if (stages == 6) {
                double a = this->constraints.maxAcceleration;
                double b = 3 * pow(this->constraints.maxAcceleration,2) / this->constraints.maxJerk;
                double c = 2 * pow(this->constraints.maxAcceleration,3) / this->constraints.maxJerk / this->constraints.maxJerk - this->distance;
                QuadraticResult t2Candidate = quadraticFormula(a, b, c);

                this->timePhase[0] = this->timePhase[2] = this->timePhase[4] = this->timePhase[6] = this->constraints.maxAcceleration / this->constraints.maxJerk;
                this->timePhase[1] = this->timePhase[5] = std::max(t2Candidate.first, t2Candidate.second);
                this->timePhase[3] = 0;
            }
        }

        void setDistancePhase(int stages) {
            if (stages == 4) {
                this->distPhase[0] = this->distPhase[6] = pow(this->timePhase[0],3) * this->constraints.maxJerk / 6;
                this->distPhase[1] = this->distPhase[3] = this->distPhase[5] = 0;
                this->distPhase[2] = this->distPhase[4] = 0.5 * this->distance - this->distPhase[0];
            }
            else if (stages == 5) {
                this->distPhase[0] = this->distPhase[6] = pow(this->timePhase[0],3) * this->constraints.maxJerk / 6;
                this->distPhase[1] = this->distPhase[5] = 0;
                this->distPhase[2] = this->distPhase[4] = this->constraints.maxVelocity * this->timePhase[0] - this->distPhase[0];
                this->distPhase[3] = this->distance - this->distPhase[0] * 2 - this->distPhase[2] * 2;
            }
            else if (stages == 6) {
                this->distPhase[0] = this->distPhase[6] = this->constraints.maxJerk * this->timePhase[0] * this->timePhase[0] * this->timePhase[0] / 6;
                this->distPhase[1] = this->distPhase[5] = this->velPhase[1] * this->timePhase[1] + 0.5 * this->accPhase[1] * this->timePhase[1] * this->timePhase[1];
                this->distPhase[2] = this->distPhase[4] = this->velPhase[2] * this->timePhase[2] + 0.5 * this->accPhase[2] * this->timePhase[2] * this->timePhase[2] - this->constraints.maxJerk * pow(timePhase[2],3) / 6;
                this->distPhase[3] = 0;
            }
        }

        void setVelocityPhase(int stages) {
            if (stages == 4) {
                this->velPhase[0] = 0;
                this->velPhase[1] = this->velPhase[2] = this->velPhase[5] = this->velPhase[6] = 0.5 * this->constraints.maxJerk * pow(this->timePhase[0],2); 
                this->velPhase[3] = this->velPhase[4] = this->velPhase[1] * 2;
            }
            else if (stages == 5) {
                this->velPhase[0] = 0;
                this->velPhase[1] = this->velPhase[2] = this->velPhase[5] = this->velPhase[6] = this->constraints.maxVelocity / 2;
                this->velPhase[3] = this->velPhase[4] = this->constraints.maxVelocity;
            }
            else if (stages == 6) {
                this->velPhase[0] = 0;
                this->velPhase[1] = this->velPhase[6] = 0.5 * this->constraints.maxJerk * this->timePhase[0] * this->timePhase[0];
                this->velPhase[2] = this->velPhase[5] = this->velPhase[1] + this->constraints.maxAcceleration * this->timePhase[1];
                this->velPhase[3] = this->velPhase[4] = this->velPhase[2] + this->accPhase[2] * this->timePhase[2] - 0.5 * this->constraints.maxJerk * pow(this->timePhase[2],2);
            }
        }

        void setAccelerationPhase(int stages) {
            if (stages == 4) {
                this->accPhase[0] = this->accPhase[3] = this->accPhase[4] = 0;
                this->accPhase[1] = this->accPhase[2] = this->constraints.maxJerk * this->timePhase[0];
                this->accPhase[5] = this->accPhase[6] = -this->accPhase[1];
            }
            else if (stages == 5) {
                this->accPhase[0] = this->accPhase[3] = this->accPhase[4] = 0;
                this->accPhase[1] = this->accPhase[2] = this->constraints.maxJerk * this->timePhase[0];
                this->accPhase[5] = this->accPhase[6] = -this->accPhase[1];
            }
            else if (stages == 6) {
                this->accPhase[0] = this->accPhase[3] = this->accPhase[4] = 0;
                this->accPhase[1] = this->accPhase[2] = this->constraints.maxJerk * this->timePhase[0];
                this->accPhase[5] = this->accPhase[6] = -this->accPhase[1];
            }
            else { // full s curve
                this->accPhase[0] = this->accPhase[3] = this->accPhase[4] = 0;
                this->accPhase[1] = this->accPhase[2] = this->constraints.maxAcceleration;
                this->accPhase[2] = this->constraints.maxAcceleration;
                this->accPhase[5] = -this->constraints.maxAcceleration;
                this->accPhase[6] = -this->constraints.maxAcceleration;
            }
        }

    public:
        void setConstraints(float maxVelocity, float maxAcceleration, float maxDeceleration, float maxJerk) {
            this->constraints.maxVelocity = maxVelocity;
            this->constraints.maxAcceleration = maxAcceleration;
            this->constraints.maxDeceleration = maxDeceleration;
            this->constraints.maxJerk = maxJerk;
        }

        void updateConstraints() {
            this->isReversed = this->distance < 0;
            this->distance = abs(this->distance);

            float time = this->constraints.maxAcceleration / this->constraints.maxJerk; // Calculates the time to reach maximum acceleration with the max rate of change of acceleration (jerk).

            if (this->constraints.maxJerk * pow(time,2) >= this->constraints.maxVelocity) { //  If max velocity can't be reached within the max jerk and acceleration.
                this->fullAcceleration = false;
                float t1 = sqrt(this->constraints.maxVelocity / this->constraints.maxJerk);
                this->minDistance = this->constraints.maxJerk * pow(t1,3) * 2; // Calculates the min distance for full acceleration
                this->fullDistance = this->minDistance;
            }
            else {
                this->fullAcceleration = true;
                float t1 = this->constraints.maxAcceleration / this->constraints.maxJerk;
                this->minDistance = this->constraints.maxJerk * pow(t1,3) * 2; // Calculates the min distance for full acceleration

                 // Calculates the full distance for full acceleration
                float t2 = (this->constraints.maxVelocity - (this->constraints.maxJerk * pow(t1,2))) / this->constraints.maxAcceleration;
                this->fullDistance = (0.5 * this->constraints.maxJerk * pow(t1,2)) * t2 + 0.5 * (this->constraints.maxAcceleration) * pow(t2,2);
                this->fullDistance += this->constraints.maxVelocity * t1;
                this->fullDistance *= 2;
            }
        }

        void setPhases() {
            setJerkPhase();
            
            if (this->distance < this->minDistance) { //  To smoothly accelerate and decelerate within a short distance
                setTimePhase(4);
                setDistancePhase(4);
                setVelocityPhase(4);
                setAccelerationPhase(4);

            }
            else if (!this->fullAcceleration) { // When reaching max acceleration isn't necessary or feasible
                setTimePhase(5);
                setDistancePhase(5);
                setVelocityPhase(5);
                setAccelerationPhase(5);
            }
            else if (this->distance < this->fullDistance) { // Allows for a smoother transition across the distance range, not necessitating the entire available motion range
                setTimePhase(6);
                setAccelerationPhase(6);
                setVelocityPhase(6);
                setDistancePhase(6);
            }
            else { // Maximum precision and control over a more extensive travel distance
                setAccelerationPhase(7);
                
                this->velPhase[0] = 0;
                this->velPhase[3] = this->constraints.maxVelocity;
                this->velPhase[4] = this->constraints.maxVelocity;
                this->timePhase[0] = this->timePhase[2] = this->timePhase[4] = this->timePhase[6] = this->constraints.maxAcceleration / this->constraints.maxJerk;
                this->velPhase[1] = this->velPhase[6] = 0.5 * this->constraints.maxJerk * this->timePhase[0] * this->timePhase[0];
                this->timePhase[1] = this->timePhase[5] = (this->constraints.maxVelocity - this->velPhase[1]*2) / this->constraints.maxAcceleration;
                this->velPhase[2] = this->velPhase[5] = this->velPhase[1] + this->accPhase[1] * this->timePhase[1];
                this->distPhase[0] = this->distPhase[6] = this->constraints.maxJerk * pow(this->timePhase[0],3) / 6;
                this->distPhase[1] = this->distPhase[5] = this->velPhase[1] * this->timePhase[1] + 0.5 * this->accPhase[1] * pow(this->timePhase[1],2);
                this->distPhase[2] = this->distPhase[4] = this->velPhase[2] * this->timePhase[2] + 0.5 * this->accPhase[2] * pow(this->timePhase[2],2) - this->constraints.maxJerk * pow(this->timePhase[2],3) / 6;
                this->timePhase[3] = (this->distance - 2 * (this->distPhase[0] + this->distPhase[1] + this->distPhase[2])) / this->constraints.maxVelocity;
                this->distPhase[3] = this->velPhase[3] * this->timePhase[3];
            }

            for (int i = 1; i < 7; i++) {
                this->timePhase[i] += this->timePhase[i-1];
                this->distPhase[i] += this->distPhase[i-1];
            }
        }

        float getPosition(float time) {
            float pt = 0;
            
            if (time < 0) {
                pt = 0;
            }
            else if (time >= this->timePhase[6]) {
                pt = this->distance;
            }
            else if (time < timePhase[0]) { // Is in the jerk phase of the S-curve.
                pt = constraints.maxJerk * pow(time, 3) / 6; // The robot’s position during this phase is the integral of the jerk
            }
            else {
                int index = 1;

                for (int i = 1; i < 7; i++) { // Determines the current phase iterating through the time phases.
                    if (time < this->timePhase[i]) {
                        index = i;
                        break;
                    }
                }

                float p0 = this->distPhase[index-1];
                float v0 = this->velPhase[index];
                float t = time - this->timePhase[index-1];
                float a0 = this->accPhase[index];
                float j = this->jerkPhase[index];

                pt = p0 + (v0 * t) + (0.5 * a0 * pow(t, 2)) + (j * pow(t, 3) / 6); // P(t) = P0 + V0*t + 1/2*A0*t^2 + 1/6*J*t^3
            }

            return this->isReversed ? -pt : pt; 
        }

        float getVelocity(float time) {
            float vel = 0;
            
            if(time < 0 || time >= this->timePhase[6]) {
                vel = 0;
            }
            else if (time < this->timePhase[0]) {
                vel = 0.5 * this->constraints.maxJerk * pow(time,2); // Calculates velocity under constant acceleration V(t) = 1/2*A0*t^
            }
            else {
                int index = 1;

                for(int i = 1; i < 7; i++){
                    if(time < this->timePhase[i]){
                        index = i;
                        break;
                    }
                }

                float t = time - this->timePhase[index-1];
                vel = this->velPhase[index] + (this->accPhase[index] * t )+ (this->jerkPhase[index] * pow(t,2) * 0.5);
            }

            return this->isReversed ? -vel : vel;  
        }

        float getAcceleration(float time) {
            float acc = 0;
            
            if(time < 0 || time >= this->timePhase[6]){
                acc = 0;
            }
            else if(time < this->timePhase[0]){
                acc = this->constraints.maxJerk * time;
            }
            else{
                int index = 1;
                for(int i = 1; i < 7; i++){
                    if(time < this->timePhase[i]){
                        index = i;
                        break;
                    }
                }
                float t = time - this->timePhase[index-1];
                acc = this->accPhase[index] + (this->jerkPhase[index] * t);
            }

            return this->isReversed ? -acc : acc;  
        }

        std::vector<float> getVelocityVector(float distance, float steps) {
            std::vector<float> velocities;

            this->distance = distance;

            this->updateConstraints();
            this->setPhases();

            for(float i = 0; i <= this->timePhase[6]; i+=steps) {
                velocities.push_back(getVelocity(i));
            }

            return velocities;
        }

        std::vector<float> getAccelerationVector(float distance, float steps) {
            std::vector<float> accelerations;

            this->distance = distance;

            this->updateConstraints();
            this->setPhases();

            for(float i = 0; i <= this->timePhase[6]; i+=steps) {
                accelerations.push_back(getAcceleration(i));
            }

            return accelerations;
        }

        std::vector<float> getPositionVector(float distance, float steps) {
            std::vector<float> positions;

            this->distance = distance;

            this->updateConstraints();
            this->setPhases();

            for(float i = 0; i <= this->timePhase[6]; i+=steps) {
                positions.push_back(getPosition(i));
            }

            return positions;
        }
};

int main() {
    S_CurveProfile profile;

    profile.setConstraints(10,10,15,20);

    std::vector<float> velocities = profile.getVelocityVector(20,0.1);
}
