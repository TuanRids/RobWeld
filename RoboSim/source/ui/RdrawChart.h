#pragma once


#include <cmath>

#include "IUIComponent.h"

namespace nui
{
    class RdrawChart: public IUIComponent
    {
    private:
        bool dChartFlag{ false };
        ImU32 FontCol;


        void DrawBarLineChart();

    public:
        // Constructor
        RdrawChart();
        ~RdrawChart();

        void render();
        void setChartFlag(bool flag){dChartFlag = flag;}
    };
}
