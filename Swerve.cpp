// Online C++ compiler to run C++ program online
// to test and create unit tests, use this https://www.chiefdelphi.com/uploads/default/original/3X/0/2/02eeda134c9afd5cc57c28b554ca8c20632467ab.xls

// Citations:
// https://www.chiefdelphi.com/t/calculating-odometry-of-a-swerve-drive/160043/6

// https://www.first1684.com/uploads/2/0/1/6/20161347/chimiswerve_whitepaper__2_.pdf

// https://www.chiefdelphi.com/uploads/default/original/3X/e/f/ef10db45f7d65f6d4da874cd26db294c7ad469bb.pdf

// there's definitelly more

// everything should be in radians until the end

#include <iostream>
#include <chrono>
#include <numbers>
#include <cmath>
#include <vector>


const double CYCLE_TIME = 0.02;
const double startingTime = 0.0;

const int    MODULES_NUMBER = 4;

const double CHASSIS_LENGTH = 25;
const double CHASSIS_WIDTH  = 25;

const double START;



class Vector
{
    public:
        static Vector FromCoords(double x, double y)
        {
            return Vector(
                x, y, 
                std::sqrt(std::pow(x,2) + std::pow(y,2)), std::atan2(x,y)
            );
        }
        
        static Vector FromMagTheta(double magnitude, double theta)
        {
            return Vector(
                std::cos(theta) * magnitude, std::sin(theta) * magnitude,
                magnitude, theta
            );
        }
        
        void RotateBy(double theta)
        {
            m_x = m_x * std::cos(theta) + 
                  m_y * std::sin(theta),
                    
            m_y = m_y * std::cos(theta) -
                  m_x * std::sin(theta);
            
            m_magnitude = std::sqrt(std::pow(m_x,2) + std::pow(m_y,2));
        }
        
        Vector operator*(double scalar)
        {
            return Vector::FromCoords(
                m_x * scalar,
                m_y * scalar
            );
        }
        
        void operator*=(double scalar)
        {
            
            m_x *= scalar;
            m_y *= scalar;
            
            m_magnitude = std::sqrt(std::pow(m_x,2) + std::pow(m_y,2));
        }
        
        double m_x;
        double m_y;
        
        double m_magnitude;
        double m_theta;
        
    private:
        Vector(double x,        double y, 
               double magnitude, double theta) 
            : m_x        {x},         m_y    {y},
              m_magnitude{magnitude}, m_theta{theta}
        {}
};

class SwerveState
{
    public:
        inline SwerveState(Vector speedVec) 
            : m_speedVec{speedVec}, m_thetaSpeed{0.0}
        {}
        
        inline SwerveState(Vector speedVec, double thetaSpeed) 
            : m_speedVec{speedVec}, m_thetaSpeed{thetaSpeed}
        {}
        
        inline SwerveState() {} // empty contructor, all values are 0
        
        inline void operator*=(const double factor)
        {
            m_speedVec   *= factor;
            m_thetaSpeed *= factor;
        } // multiplied by scalar
        
        inline SwerveState operator*(const double factor)
        {
            return SwerveState(m_speedVec * factor,
                               m_thetaSpeed * factor);
        } // multiplies by scalar and returns
        
        void FieldCentricConversion(double currentTheta)
        {
            m_speedVec.RotateBy(currentTheta);
        }
        
        Vector m_speedVec   = Vector::FromCoords(0,0);
        double m_thetaSpeed = 0.0; // radians per sec
};

typedef Vector ModuleSpeeds;

using SwerveSpeeds = std::vector<ModuleSpeeds>;

class Position
{
    public:
        inline Position(double x, double y) 
            : m_x{x}, m_y{y}, m_theta{0.0}
        {}
        
        inline Position(double x, double y, double theta) 
            : m_x{x}, m_y{y}, m_theta{theta}
        {}
        
        inline Position operator+(Position const& obj)
        {
            return Position{m_x + obj.m_x, 
                            m_y + obj.m_y,
                            m_theta + obj.m_theta
            };
        }
        
        inline Position operator+(SwerveState const& obj)
        {
            return Position{m_x + obj.m_speedVec.m_x, 
                            m_y + obj.m_speedVec.m_y,
                            m_theta += obj.m_thetaSpeed
            };
        }
        
        void operator+=(Position const& obj)
        {
            m_x += obj.m_x;
            m_y += obj.m_y;
            m_theta += obj.m_theta;
        }
        
        void operator+=(SwerveState const& obj)
        {
            m_x += obj.m_speedVec.m_x;
            m_y += obj.m_speedVec.m_y;
            m_theta += obj.m_thetaSpeed;
        }
        
        double m_x;
        double m_y;
        double m_theta;
}; // A position

template <int modules>
class Odometry
{
    public:
        inline Odometry(double timeStep, double length, double width)
            : m_currentTime  {0.0},
              m_timeStep     {timeStep},
              m_position     {0.0, 0.0},
              m_chassisLength{length},
              m_chassisWidth{width}
        {}
        
        inline void Periodic(SwerveState currentState)
        {
            currentState.FieldCentricConversion(
                m_position.m_theta + (currentState.m_thetaSpeed * m_timeStep)
            );
            
            m_position    += currentState * m_timeStep;
            m_currentTime += m_timeStep;
        }
        
        inline void Periodic(SwerveSpeeds currentSpeeds)
        {
        	const double FR_B = std::sin(currentSpeeds[0].m_theta) * currentSpeeds[0].m_magnitude;
        	const double FR_C = std::cos(currentSpeeds[0].m_theta) * currentSpeeds[0].m_magnitude;
        	
        	const double FL_B = std::sin(currentSpeeds[1].m_theta) * currentSpeeds[1].m_magnitude;
        	const double FL_D = std::cos(currentSpeeds[1].m_theta) * currentSpeeds[1].m_magnitude;
        
        	const double BR_A = std::sin(currentSpeeds[2].m_theta) * currentSpeeds[2].m_magnitude;
        	const double BR_C = std::cos(currentSpeeds[2].m_theta) * currentSpeeds[2].m_magnitude;
        
        	const double BL_A = std::sin(currentSpeeds[3].m_theta) * currentSpeeds[3].m_magnitude;
        	const double BL_D = std::cos(currentSpeeds[3].m_theta) * currentSpeeds[3].m_magnitude;
        
        	const double A = (BR_A + BL_A) / 2.0;
        	const double B = (FR_B + FL_B) / 2.0;
        	const double C = (FR_C + BR_C) / 2.0;
        	const double D = (FL_D + BL_D) / 2.0;
        
        	const double omega1 = (B - A) / m_chassisLength;
        	const double omega2 = (C - D) / m_chassisWidth;
        	const double omega = (omega1 + omega2) / 2.0;
        
        	const double STR1 = omega * (m_chassisLength / 2.0) + A;
        	const double STR2 = -omega * (m_chassisLength / 2.0) + B;
        	const double FWD1 = omega * (m_chassisWidth / 2.0) + C;
        	const double FWD2 = -omega * (m_chassisWidth / 2.0) + D;
        
        	const double STR = (STR1 + STR2) / 2.0;
        	const double FWD = (FWD1 + FWD2) / 2.0;

            
            // Getting the forward and strafe values
            // basically unit circle lol
            this->Periodic(SwerveState{
                Vector::FromCoords(FWD, STR),
                0.0
            });
            
        }
        
        void PrintPosition()
        {
            const double angle = (m_position.m_theta * 180) / 3.141;
            
            std::cout << "Timestamp  " << m_currentTime      << std::endl 
                      << "xPosition  " << m_position.m_x     << std::endl 
                      << "yPosition  " << m_position.m_y     << std::endl
                      << "thetaAngle " << angle              << std::endl
                      << std::endl;
        }
        
    private:
        double   m_currentTime;
        double   m_timeStep;
        Position m_position;
        
        double m_chassisLength;
        double m_chassisWidth;
};

template <int modulesNum>
class Swerve
{
    public:
        inline Swerve(double length, double width)
            : m_chassisLength{length},
              m_chassisWidth{width},
              m_modulePositionConstant{
                  std::sqrt(std::pow(m_chassisLength, 2) +
                            std::pow(m_chassisWidth,  2) )
              }
        {}
        
        inline SwerveSpeeds Drive(SwerveState inputs)
        {
            double forward = inputs.m_speedVec.m_x;
            double strafe  = inputs.m_speedVec.m_y;
            double rotate  = inputs.m_thetaSpeed;
            rotate = 0; // Dont support rotation yet
            
            double A = strafe  - rotate * 
                (m_chassisLength / m_modulePositionConstant);
            
            double B = strafe  + rotate * 
                (m_chassisLength / m_modulePositionConstant);
            
            double C = forward - rotate * 
                (m_chassisWidth  / m_modulePositionConstant);
            
            double D = forward + rotate * 
                (m_chassisWidth  / m_modulePositionConstant);
                
            return {
                Vector::FromMagTheta(
                    std::sqrt(B * B + C * C), std::atan2(B, C)),
                Vector::FromMagTheta(
                    std::sqrt(B * B + D * D), std::atan2(B, D)),
                Vector::FromMagTheta(
                    std::sqrt(A * A + D * D), std::atan2(A, D)),
                Vector::FromMagTheta(
                    std::sqrt(A * A + C * C), std::atan2(A, C))
            };
        }
        
        double m_chassisLength;
        double m_chassisWidth;
        
        double m_modulePositionConstant;
};

int main() {
    // settings
    double cycleTime;
    double startingTime;
    
    int    modulesNum = 4;
    
    double chassisLength = 25;
    double chassisWidth  = 25;
    
    double startingPosition;
    
    Swerve<modulesNum>   swerve  {chassisLength, chassisWidth};
    Odometry<modulesNum> odometry{cycleTime, chassisLength, chassisWidth};
    
    odometry.PrintPosition();
    
    while (true)
    {
        double xInput;
        double yInput;
        std::cout << "x input: " << std::endl;
        std::cin  >> xInput;
        std::cout << "y input: " << std::endl;
        std::cin  >> yInput;
        
        odometry.Periodic(swerve.Drive(
            SwerveState{ Vector::FromCoords(xInput, yInput), 0.0 }));
        odometry.PrintPosition();
    }

    return 0;
}