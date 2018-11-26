#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>
#include <cmath>
#include <opencv2/highgui/highgui.hpp>


extern "C" void ip_init(void);
extern "C" struct ip_res *ip_process(void);

void ip_init(void) {

}

double img_center_pt;
bool lline_found = false;
bool rline_found = false;
bool sline_found = false;
double rline_slope;
double lline_slope;
double sline_slope;
cv::Point raxis_intersection;
cv::Point laxis_intersection;
cv::Point saxis_intersection;

cv::Mat img_edge_detector(cv::Mat&); 
cv::Mat mask_image(cv::Mat&);
std::vector<cv::Vec4i> find_lines(cv::Mat&);
std::vector<std::vector<cv::Vec4i> > classify_lines(std::vector<cv::Vec4i>&, cv::Mat&);
std::vector<cv::Point> linear_regression(std::vector<std::vector<cv::Vec4i>>&, cv::Mat&);
void plotLane(cv::Mat&, std::vector<cv::Point>&);


cv::Mat img_edge_detector(cv::Mat& image) {
    cv::Mat edge_img;

    //smoothing to reduce noise úsing Gaussian filter (most used but not fastest)
    cv::GaussianBlur(image, edge_img, cv::Size(3, 3), 0, 0);
    cv::cvtColor(edge_img, edge_img, cv::COLOR_RGB2GRAY);
    
    int thresh_value = 140;
    int max_binary_value =230;

    //Segmentation. "Assign a label to every pixel in an image such that pixels with the
    // same label share certain characteristics.
    //Threshold: simplest and not expensive
    // Otsu better than binary but maybe much more expensive
    cv::threshold(edge_img, edge_img, thresh_value, max_binary_value, cv::THRESH_OTSU);
    cv::imshow("threshold", edge_img);
    
    /*float data[3] = { -1, 0, 1};
    cv::Mat kernel(1, 3, CV_32F, data);
    
    //recommended values when using filter2D
    cv::Point anchor  = cv::Point(-1, -1);
    const int DELTA = 0;
    const int DDEPTH = -1;*/

    //cv::warpPerspective(denoised_image, denoised_image, kernel, denoised_image.size());
    //cv::inRange(denoised_image, cv::Scalar(1), cv::Scalar(255), denoised_image);

  
    cv::Canny(edge_img, edge_img, 100, 150, 3);
	//Supposedly, a more simple alternative for filter2D. But it has not shown improvement
    //cv::filter2D(edge_img, edge_img, DDEPTH, kernel, anchor, DELTA, cv::BORDER_DEFAULT);
    cv::imshow("CannyEdges: ", edge_img);

    return edge_img;
}

cv::Mat mask_image(cv::Mat& image) {
    cv::Mat mask(cv::Mat::zeros(image.size(), image.type()));
    
    /* Region Of Interes */
    float ROI_y_start = image.rows;
    float ROI_y_end = (image.rows / 2);
    std::cout << "y:" << ROI_y_end << "\n";
    std::cout << "y:" << ROI_y_start << "\n";
    float ROI_x1 = 0.08 * image.cols;
    std::cout << "x1:" << ROI_x1 << "\n";
    float ROI_x2 = 0.35 * image.cols;
    std::cout << "x2:" << ROI_x2 << "\n";
    float ROI_x3 = 0.65 * image.cols;
    std::cout << "x3:" << ROI_x3 << "\n";
    float ROI_x4 = 0.92 * image.cols;
    std::cout << "x4:" << ROI_x4 << "\n";
    const int pts_amount = 4;
    cv::Point p1(ROI_x1, ROI_y_start), p2(ROI_x2, ROI_y_end), p3(ROI_x3, ROI_y_end), p4(ROI_x4, ROI_y_start);
    cv::Point ROI[4] = {p1, p2, p3, p4};
    
    cv::fillConvexPoly(mask, ROI, pts_amount, cv::Scalar(255, 0, 0));
    cv::bitwise_and(image, mask, image);
    return image;
}

std::vector<cv::Vec4i> find_lines(cv::Mat& image) {
    std::vector<cv::Vec4i> lines; //Vec<int,4>

    double rho = 1;
    double theta = CV_PI / 180;
    int threshold = 130; //20
    double minLineLength = 20;//20
    double maxLineGap = 75; //30

    cv::HoughLinesP(image, lines, rho, theta, threshold, minLineLength, maxLineGap);

    return lines;
}

std::vector<std::vector<cv::Vec4i> > classify_lines(std::vector<cv::Vec4i>& lines, cv::Mat& image) {
    std::vector<std::vector<cv::Vec4i> > classified_lines(3); //3 when stop
    cv::Point start;
    cv::Point end;
    std::vector<float> slopes;
    std::vector<cv::Vec4i> right_lines, left_lines, stop_lines;

    for (auto point : lines) {
        start = cv::Point(point[0], point[1]);
        end = cv::Point(point[2], point[3]);
        double slope = (static_cast<double>(end.y) - static_cast<double>(start.y)) / (static_cast<double>(end.x) - static_cast<double>(start.x));
        slopes.push_back(slope);
    }

    img_center_pt = static_cast<double>((image.cols / 2));
    size_t x = 0;
    while (x < lines.size()) {
        start = cv::Point(lines[x][0], lines[x][1]);
        end = cv::Point(lines[x][2], lines[x][3]);

        
  	if (slopes[x] == 0 || (start.x < img_center_pt && end.x > img_center_pt)) {
        	stop_lines.push_back(lines[x]);
        	stop_found = true;
        }
  	else if (slopes[x] > 0 && end.x > img_center_pt && start.x > img_center_pt) {
            right_lines.push_back(lines[x]);
            rline_found = true;
        }
        else if (slopes[x] < 0 && end.x < img_center_pt && start.x < img_center_pt) {
            left_lines.push_back(lines[x]);
            lline_found = true;
        }
        x++;
    }

    classified_lines[0] = right_lines;
    classified_lines[1] = left_lines;
    classified_lines[2] = stop_lines;

    return classified_lines;
}

std::vector<cv::Point> linear_regression(std::vector<std::vector<cv::Vec4i>>& lines, cv::Mat& image) {
    std::vector<cv::Point> points(6); //6 when stop
    cv::Point start;
    cv::Point end;
    cv::Vec4f right_line;
    cv::Vec4f left_line;
    cv::Vec4f stop_line;
    std::vector<cv::Point2f> right_pts;
    std::vector<cv::Point2f> left_pts;
    std::vector<cv::Points> stop_pts;

    if (rline_found) {
        for (auto points : lines[0]) {
            start = cv::Point(points[0], points[1]);
            end = cv::Point(points[2], points[3]);

            right_pts.push_back(start);
            right_pts.push_back(end);
            std::cout << "right_ " << start << "-" << end << "\n";
        }
        if (right_pts.size() > 0) {
            cv::fitLine(right_pts, right_line, CV_DIST_L2, 0, 0.01, 0.01);
	    std::cout << "fitLine has been passed: \n";
            rline_slope = right_line[1] / right_line[0];
            raxis_intersection = cv::Point(right_line[2], right_line[3]);
        }
    }
    if (lline_found) {
        for (auto point : lines[1]) {
            start = cv::Point(point[0], point[1]);
            end = cv::Point(point[2], point[3]);

            left_pts.push_back(start);
            left_pts.push_back(end);
        }
        if (left_pts.size() > 0) {
            cv::fitLine(left_pts, left_line, CV_DIST_L2, 0, 0.01, 0.01);
            lline_slope = left_line[1] / left_line[0];
            laxis_intersection = cv::Point(left_line[2], left_line[3]);
        }
    }
    if (sline_found) {
        for (auto j : lines[2]) {
            start = cv::Point(j[0], j[1]);
            end = cv::Point(j[2], j[3]);

            stop_points.push_back(start);
            stop_points.push_back(end);
        }

        if (stop_points.size() > 0) {
            cv::fitLine(stop_pts, stop_line, CV_DIST_L2, 0, 0.01, 0.01);
            sline_slope = stop_line[1] / stop_line[0];
            saxis_intersection = cv::Point(stop_line[2], stop_line[3]);
        }
    }

    int lines_start_y = image.rows;
    int lines_end_y = 0.6 * image.rows;
    int sline_start_x = 0.20*image.cols;
    int sline_end_x = 0.80*image.cols;

    double rline_start_x = ((lines_start_y - raxis_intersection.y) / rline_slope) + raxis_intersection.x;
    double rline_end_x = ((lines_end_y - raxis_intersection.y) / rline_slope) + raxis_intersection.x;

    double lline_start_x = ((lines_start_y - laxis_intersection.y) / lline_slope) + laxis_intersection.x;
    double lline_end_x = ((lines_end_y - laxis_intersection.y) / lline_slope) + laxis_intersection.x;

    double sline_start_y = (sline_slope * (sline_start_x - saxis_intersection.x)) + saxis_intersection.y;
    double sline_end_y = (sline_slope * (sline_end_x - saxis_intersection.x)) + saxis_intersection.y;

    points[0] = cv::Point(rline_start_x, lines_start_y);
    points[1] = cv::Point(rline_end_x, lines_end_y);
    points[2] = cv::Point(lline_start_x, lines_start_y);
    points[3] = cv::Point(lline_end_x, lines_end_y);
    points[4] = cv::Point(sline_start_x, sline_start_y);
    points[5] = cv::Point(sline_end_x, sline_end_y);
    
    return points;
}

void plotLane(cv::Mat& original_img, std::vector<cv::Point>& points) {
    std::vector<cv::Point> polygon_pts;
    cv::Mat lane;

    original_img.copyTo(lane);
    polygon_pts.push_back(points[0]);
    polygon_pts.push_back(points[1]);
    polygon_pts.push_back(points[3]);
    polygon_pts.push_back(points[2]);

    cv::fillConvexPoly(lane, polygon_pts, cv::Scalar(0, 0, 255), CV_AA, 0);
    cv::addWeighted(lane, 0.3, original_img, 1.0 - 0.3, 0, original_img);

    cv::line(original_img, points[0], points[1], cv::Scalar(255, 0, 0), 5, CV_AA);
    cv::line(original_img, points[2], points[3], cv::Scalar(255, 0, 0), 5, CV_AA);
    cv::line(original_img, points[4], points[5], cv::Scalar(0, 0, 0), 5, CV_AA);
    cv::circle(original_img, points[0], 6, cv::Scalar(0, 0, 0), CV_FILLED);
    cv::circle(original_img, points[1], 6, cv::Scalar(0, 0, 0), CV_FILLED);
    cv::circle(original_img, points[2], 6, cv::Scalar(0, 0, 0), CV_FILLED);
    cv::circle(original_img, points[3], 6, cv::Scalar(0, 0, 0), CV_FILLED);
    cv::circle(original_img, points[4], 6, cv::Scalar(0, 0, 255), CV_FILLED);
    cv::circle(original_img, points[5], 6, cv::Scalar(0, 0, 255), CV_FILLED);

    rline_found = false;
    lline_found = false;
    sline_found = false;

    std::string message = "De baxar dina byxor!!!";
    cv::putText(original_img, message, cv::Point(0.1*original_img.cols, 0.2*original_img.rows), cv::FONT_HERSHEY_TRIPLEX, 2, cvScalar(0, 0, 0), 5, CV_FILLED);

    /*Test: Getting distance between a specified line point and the camera*/ 
    cv::Point2f stop_center_pt = cv::Point((points[4].x + points[5].x)/2, (points[4].y + points[5].y)/2);
    std::cout << "stopLine center pt: " << stop_center_pt << "\n";
    cv:circle(original_img, stop_center_pt, 6, cv::Scalar(0, 0, 255), CV_FILLED);
    std::cout << "center: " << img_center_pt << "\n";

    float stop_dist = std::sqrt(pow(stop_center_pt.x - img_center_pt, 2) + pow(stop_center_pt.y - original_img.rows, 2));
	std::string distance = std::to_string(stop_dist);

    cv::putText(original_img, distance, cv::Point(0.1*original_img.cols, 0.3*original_img.rows), cv::FONT_HERSHEY_TRIPLEX, 1, cvScalar(255, 0, 0), 5);
}


struct ip_res *ip_process(void) {
    std::cout << "hej från c++" << std::endl;

    cv::VideoCapture cap(-1);
    if (!cap.isOpened()) {
        std::cout << "Error: Camera not found\n";
        return NULL;
    }
  
    cap.set(CV_CAP_PROP_FRAME_WIDTH, 352);
    cap.set(CV_CAP_PROP_FRAME_HEIGHT,240);
    
    std::cout << "Width: " << cap.get(CV_CAP_PROP_FRAME_WIDTH)<< "\n";
    std::cout << "Height: " << cap.get(CV_CAP_PROP_FRAME_HEIGHT)<< "\n";

    //std::cout << "FPS: " << cap.get(CV_CAP_PROP_FPS) << "\n";
    //cap.set(CV_CAP_PROP_FPS, 60);
    //std::cout << "FPS2: " << cap.get(CV_CAP_PROP_FPS) << "\n";

    
    cv::Mat frame;
    cv::Mat denoised_image;
    cv::Mat edges_image;
    cv::Mat masked_image;
    cv::Mat img_lines;
    std::vector<cv::Vec4i> lines;
    std::vector<std::vector<cv::Vec4i> > lr_lines;
    std::vector<cv::Point> lane;
    
    cv::namedWindow("Lane", CV_WINDOW_AUTOSIZE);

    /*while (true) {
        //grab and decode latest frame 
        while (cap.get(CV_CAP_PROP_FRAME_COUNT) > 0) cap.grab();
        bool succ = cap.retrieve(frame);
        if (succ != true) {
            break;
        }*/
    while (true) {
	
	cap.read(frame);
	if (frame.empty()) {
	    std::cout << "Error: Empty frame\n";
	    break;
	}

        edges_image = img_edge_detector(frame);
        masked_image = mask_image(edges_image);
        cv::imshow("mask", masked_image);

        lines = find_lines(masked_image);
        
        if (!lines.empty()) {
            lr_lines = classify_lines(lines, edges_image);

            lane = linear_regression(lr_lines, frame);

            plotLane(frame, lane);

        }
            cv::imshow("Lane", frame);
            int k = cv::waitKey(1);

            if (k == 27)
         	break;
        }
    frame.release();
    cv::destroyAllWindows();
    return NULL;
}
