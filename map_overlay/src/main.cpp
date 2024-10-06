#include <opencv2/opencv.hpp>
#include <iostream>

int main(int argc, char** argv) {
    if (argc != 4) {
        std::cerr << "Usage: " << argv[0] << " <path_to_map1> <path_to_map2> <alpha>" << std::endl;
        return -1;
    }

    std::string map1_path = argv[1];
    std::string map2_path = argv[2];
    double alpha = std::stod(argv[3]);

    // Load the maps
    cv::Mat map1 = cv::imread(map1_path, cv::IMREAD_UNCHANGED);
    cv::Mat map2 = cv::imread(map2_path, cv::IMREAD_UNCHANGED);

    if (map1.empty() || map2.empty()) {
        std::cerr << "Error loading one of the maps." << std::endl;
        return -1;
    }

    // Resize the maps to a smaller size (e.g., half the original size)
    cv::Size newSize(map1.cols / 2.5, map1.rows / 2.5);
    cv::resize(map1, map1, newSize);

    // Calculate the new size for map2 while maintaining the aspect ratio
    double scale = 2; // Adjust this scale factor as needed
    cv::Size newMap2Size(map2.cols * scale, map2.rows * scale);
    cv::resize(map2, map2, newMap2Size);

    // Create an overlay with reduced opacity
    double beta = 1.0 - alpha;
    cv::Mat overlay = map1.clone();

    // Calculate the center position for map2
    int x_offset = (map1.cols - map2.cols) / 2;
    int y_offset = (map1.rows - map2.rows) / 2;

    // Define the region of interest (ROI) in the overlay where the second map will be placed
    cv::Rect roi(x_offset + 7, y_offset - 9, map2.cols, map2.rows);

    // Overlay the second map onto the first map within the ROI
    cv::addWeighted(map1(roi), alpha, map2, beta, 0.0, overlay(roi));

    // Save the overlay as a PNG file
    cv::imwrite("overlay.png", overlay);

    // Display the result
    cv::imshow("Overlay", overlay);
    cv::waitKey(0);

    return 0;
}
