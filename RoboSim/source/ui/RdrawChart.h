#pragma once

#include <vector>
#include <map>
#include <imgui.h>

namespace nui
{
    class RdrawChart
    {
    private:
        bool dChartFlag{ false };
    public:
        // Constructor
        RdrawChart();

        // Destructor
        ~RdrawChart();

        // Function to draw table
        void DrawTable(const std::vector<std::vector<float>>& data);

        // Function to draw bar chart
        void DrawBarChart(const std::map<std::string, std::pair<float, std::string>>& data);

        // Function to draw line chart
        void DrawLineChart(const std::map<std::string, std::pair<float, std::string>>& data);

        // draw bar&line
        void DrawBarLineChart(const std::map<std::string, std::pair<float, std::string>>& data);
        void DrawBarLineQc(const std::map<std::string, std::pair<float, std::string>>& data);


        // Function to draw pie chart
        void DrawPieChart(const std::map<std::string, std::pair<float, std::string>>& data);

        // Function to draw 3D bar chart
        void Draw3DBarChart(const std::vector<float>& data) {}

        // Function to draw 3D pie chart
        void Draw3DPieChart(const std::vector<float>& data) {}

        // Function to draw 3D line chart
        void Draw3DLineChart(const std::vector<float>& data) {}
        void render();

        void setChartFlag(bool flag)
        {
            dChartFlag = flag;
        }
    };
}
