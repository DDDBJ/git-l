#ifndef ROBOT_H_
#define ROBOT_H_

#include <memory>
#include <aris.hpp>
#include <termio.h>
#include <stdio.h>


namespace robot
{

    class TcurveDrive :public aris::core::CloneObject<TcurveDrive,aris::plan::Plan>
    {
    public:
        auto virtual prepareNrt()->void;
        auto virtual executeRT()->int;
        auto virtual collectNrt()->void;

        int scanKeyboard()
        {
        int in;
        struct termios new_settings;
        struct termios stored_settings;
        tcgetattr(0,&stored_settings);
        new_settings = stored_settings;
        new_settings.c_lflag &= (~ICANON);
        new_settings.c_cc[VTIME] = 0;
        tcgetattr(0,&stored_settings);
        new_settings.c_cc[VMIN] = 1;
        tcsetattr(0,TCSANOW,&new_settings);

        in = getchar();

        tcsetattr(0,TCSANOW,&stored_settings);
        return in;
        }

        virtual ~TcurveDrive();
        explicit TcurveDrive(const std::string &name = "motor_drive");

    private:
        double dir_;
    };

    class Coscurve :public aris::core::CloneObject<Coscurve,aris::plan::Plan>
            {
            public:
                auto virtual prepareNrt()->void;
                auto virtual executeRT()->int;
                auto virtual collectNrt()->void;

                explicit Coscurve(const std::string &name = "motor_cos_drive");

            };

    auto createControllerMotor()->std::unique_ptr<aris::control::Controller>;
    auto createPlanMotor()->std::unique_ptr<aris::plan::PlanRoot>;
}

#endif
