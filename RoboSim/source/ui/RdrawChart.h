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

        // Function to draw table
        void DrawTable(const std::vector<std::vector<float>>& data);

        // Function to draw bar chart
        void DrawBarChart(const std::map<std::string, std::pair<float, std::string>>& data);

        // Function to draw line chart
        void DrawLineChart(const std::map<std::string, std::pair<float, std::string>>& data);
        void DrawBarLineChart();
        void DrawBarLineQc(const std::map<std::string, std::pair<float, std::string>>& data);
        void DrawPieChart(const std::map<std::string, std::pair<float, std::string>>& data);

        // Function to draw 3D bar chart
        void Draw3DBarChart(const std::vector<float>& data) {}
        void Draw3DPieChart(const std::vector<float>& data) {}
        void Draw3DLineChart(const std::vector<float>& data) {}
    public:
        // Constructor
        RdrawChart();
        ~RdrawChart();

        void render();
        void setChartFlag(bool flag){dChartFlag = flag;}
    };
}
