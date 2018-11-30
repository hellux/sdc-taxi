#include "img_proc.h"

#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>
#include <cmath>
#include <opencv2/highgui/highgui.hpp>
#include <chrono>

#define WIDTH 256
#define HEIGHT 144

const int THR_TYPES[] = {cv::THRESH_BINARY, cv::THRESH_BINARY_INV,
                         cv::THRESH_TRUNC, cv::THRESH_TOZERO,
                         cv::THRESH_TOZERO_INV, cv::THRESH_MASK,
                         cv::THRESH_OTSU, cv::THRESH_TRIANGLE};
#define TRACK_MAX 255
#define TRACK_MAX_THRESH sizeof(THR_TYPES)/sizeof(*THR_TYPES)

static int thresh_value = 20;
static int thresh_type = 6;
static int hough_threshold = 7;
static int line_min_length = 0;
static int line_max_gap = 40;
static int mask_width_top = 0.85*WIDTH;
static int mask_width_bot = 1*WIDTH;
static int mask_start_y = 0.9*HEIGHT;
static int mask_end_y = 0.6*HEIGHT;

extern "C" struct ip_data *ip_init();
extern "C" void ip_destroy(struct ip_data *ip);
extern "C" void ip_process(struct ip_data *ip, struct ip_res *res);

struct ip_data {
    cv::VideoCapture *cap;
};

struct ip_data *ip_init() {
    struct ip_data *ip = (struct ip_data*)malloc(sizeof(*ip));

    ip->cap = new cv::VideoCapture(-1);

    if (!ip->cap->isOpened()) {
        std::cout << "Error: Camera not found\n";
        ip_destroy(ip);
        return NULL;
    }
  
    ip->cap->set(CV_CAP_PROP_FRAME_WIDTH, WIDTH);
    ip->cap->set(CV_CAP_PROP_FRAME_HEIGHT, HEIGHT);

#ifdef VISUAL
    cv::namedWindow("Lane", CV_WINDOW_AUTOSIZE);
    cv::namedWindow("", CV_WINDOW_AUTOSIZE);
#endif

    return ip;
}

void ip_destroy(struct ip_data *ip) {
    delete ip->cap;
    cv::destroyAllWindows();
}

cv::Mat threshold(cv::Mat& image) {
    cv::Mat edge_img;
    
    //smoothing to reduce noise úsing Gaussian filter (most used but not fastest)
    cv::GaussianBlur(image, edge_img, cv::Size(3, 3), 0, 0);
    cv::cvtColor(edge_img, edge_img, cv::COLOR_RGB2GRAY);
    
    //Segmentation. "Assign a label to every pixel in an image such that pixels with the
    // same label share certain characteristics.
    //Threshold: simplest and not expensive
    // Otsu better than binary but maybe much more expensive
    int max_binary_value = 255;
    cv::threshold(edge_img, edge_img, thresh_value, max_binary_value,
                  THR_TYPES[thresh_type]);

    return edge_img;
}

cv::Mat img_edge_detector(cv::Mat& image) {
    cv::Mat edge_img;
    cv::Canny(image, edge_img, 100, 150, 3);

    return edge_img;
}

cv::Mat mask_image(cv::Mat& image) {
    cv::Mat masked_image;
    cv::Mat mask(cv::Mat::zeros(image.size(), image.type()));

    /* Region Of Interest */
    float top_y = mask_end_y;
    float top_rx = (image.cols+mask_width_top)/2;
    float top_lx = (image.cols-mask_width_top)/2;

    float bot_y = mask_start_y;
    float bot_lx = (image.cols-mask_width_bot)/2;
    float bot_rx = (image.cols+mask_width_bot)/2;

    std::cout << "y:" << bot_y << "\n";
    std::cout << "y:" << top_y << "\n";
    std::cout << "x1:" << bot_lx << "\n";
    std::cout << "x2:" << top_rx << "\n";
    std::cout << "x3:" << top_lx << "\n";
    std::cout << "x4:" << bot_rx << "\n";

    cv::Point p0(0, HEIGHT), p1(bot_lx, bot_y), p2(top_lx, top_y),
              p3(top_rx, top_y), p4(bot_rx, bot_y), p5(WIDTH, HEIGHT);
    cv::Point roi[] = {p0, p1, p2, p3, p4, p5};
    
    cv::fillConvexPoly(mask, roi, sizeof(roi)/sizeof(*roi), cv::Scalar(255, 0, 0));
    cv::bitwise_and(image, mask, masked_image);

    return masked_image;
}

std::vector<cv::Vec4i> find_lines(cv::Mat& image) {
    std::vector<cv::Vec4i> lines; //Vec<int,4>

    double rho = 1;
    double theta = CV_PI / 180;

    cv::HoughLinesP(image, lines, rho, theta, hough_threshold, line_min_length, line_max_gap);

    return lines;
}

std::vector<std::vector<cv::Vec4i>> classify_lines(std::vector<cv::Vec4i>& lines,
                                                   cv::Mat& image) {
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

    double center_x = static_cast<double>((image.cols / 2));
    for (size_t x = 0; x < lines.size(); x++) {
        start = cv::Point(lines[x][0], lines[x][1]);
        end = cv::Point(lines[x][2], lines[x][3]);
        if (slopes[x] == 0 || (start.x < center_x && end.x > center_x)) {
            stop_lines.push_back(lines[x]);
        } else if (slopes[x] > 0 && (end.x > center_x && start.x > center_x && end.y > (0.95 * image.rows))) {
            right_lines.push_back(lines[x]);
        } else if (slopes[x] < 0 && (end.x < center_x && start.x < center_x && start.y > (0.95 * image.rows))) {
            left_lines.push_back(lines[x]);
            std::cout<<"left_line found: "<< lines[x] << "\n" ;
        }
    }

    classified_lines[0] = right_lines;
    classified_lines[1] = left_lines;
    classified_lines[2] = stop_lines;

    return classified_lines;
}

std::vector<cv::Point>
linear_regression(std::vector<std::vector<cv::Vec4i>>& lines, cv::Mat& image,
                  bool rline_found, bool lline_found, bool sline_found) {
    std::vector<cv::Point> points(6); //6 when stop
    cv::Point start;
    cv::Point end;
    cv::Vec4f right_line;
    cv::Vec4f left_line;
    cv::Vec4f stop_line;
    std::vector<cv::Point2f> right_pts;
    std::vector<cv::Point2f> left_pts;
    std::vector<cv::Point2f> stop_pts;

    double rline_slope = 0;
    double lline_slope = 0;
    double sline_slope = 0;
    cv::Point rline_intercept = {0};
    cv::Point lline_intercept = {0};
    cv::Point sline_intercept = {0};

    if (rline_found) {
        for (auto points : lines[0]) {
            start = cv::Point(points[0], points[1]);
            end = cv::Point(points[2], points[3]);

            right_pts.push_back(start);
            right_pts.push_back(end);
        }
        if (right_pts.size() > 0) {
            cv::fitLine(right_pts, right_line, CV_DIST_L2, 0, 0.01, 0.01);
            rline_slope = right_line[1] / right_line[0];
            rline_intercept = cv::Point(right_line[2], right_line[3]);
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
            lline_intercept = cv::Point(left_line[2], left_line[3]);
        }
    }
    if (sline_found) {
        for (auto point : lines[2]) {
            start = cv::Point(point[0], point[1]);
            end = cv::Point(point[2], point[3]);

            stop_pts.push_back(start);
            stop_pts.push_back(end);
        }
        if (stop_pts.size() > 0) {
            cv::fitLine(stop_pts, stop_line, CV_DIST_L2, 0, 0.01, 0.01);
            sline_slope = stop_line[1] / stop_line[0];
            sline_intercept = cv::Point(stop_line[2], stop_line[3]);
        }
    }

    int lines_start_y = image.rows;
    int lines_end_y = 0.60 * image.rows;
    int sline_start_x = 0.20*image.cols;
    int sline_end_x = 0.80*image.cols;

    double rline_start_x = ((lines_start_y - rline_intercept.y) / rline_slope) + rline_intercept.x;
    double rline_end_x = ((lines_end_y - rline_intercept.y) / rline_slope) + rline_intercept.x;

    double lline_start_x = ((lines_start_y - lline_intercept.y) / lline_slope) + lline_intercept.x;
    double lline_end_x = ((lines_end_y - lline_intercept.y) / lline_slope) + lline_intercept.x;

    double sline_start_y = (sline_slope * (sline_start_x - sline_intercept.x)) + sline_intercept.y;
    double sline_end_y = (sline_slope * (sline_end_x - sline_intercept.x)) + sline_intercept.y;

    points[0] = cv::Point(rline_start_x, lines_start_y);
    points[1] = cv::Point(rline_end_x, lines_end_y);
    points[2] = cv::Point(lline_start_x, lines_start_y);
    points[3] = cv::Point(lline_end_x, lines_end_y);
    points[4] = cv::Point(sline_start_x, sline_start_y);
    points[5] = cv::Point(sline_end_x, sline_end_y);
    
    return points;
}

void plotLane(cv::Mat& original_img, std::vector<cv::Point>& points,
              bool rline_found, bool lline_found, bool sline_found) {
    std::vector<cv::Point> polygon_pts;
    cv::Mat lane;

    original_img.copyTo(lane);
    polygon_pts.push_back(points[0]);
    polygon_pts.push_back(points[1]);
    polygon_pts.push_back(points[3]);
    polygon_pts.push_back(points[2]);

    cv::fillConvexPoly(lane, polygon_pts, cv::Scalar(0, 0, 255), CV_AA, 0);
    cv::addWeighted(lane, 0.3, original_img, 1.0 - 0.3, 0, original_img);

    if (rline_found) {
        cv::line(original_img, points[0], points[1], cv::Scalar(255, 0, 0), 5, CV_AA);
        cv::circle(original_img, points[0], 6, cv::Scalar(0, 0, 0), CV_FILLED);
        cv::circle(original_img, points[1], 6, cv::Scalar(0, 0, 0), CV_FILLED);
    }
    if (lline_found) {
	    cv::line(original_img, points[2], points[3], cv::Scalar(255, 0, 0), 5, CV_AA);
        cv::circle(original_img, points[2], 6, cv::Scalar(0, 0, 0), CV_FILLED);
        cv::circle(original_img, points[3], 6, cv::Scalar(0, 0, 0), CV_FILLED);
    }
    if (sline_found) {
	    cv::line(original_img, points[4], points[5], cv::Scalar(0, 0, 0), 5, CV_AA);
        cv::circle(original_img, points[4], 6, cv::Scalar(0, 0, 255), CV_FILLED);
        cv::circle(original_img, points[5], 6, cv::Scalar(0, 0, 255), CV_FILLED);
    }
    
    std::string message = "De baxar dina byxor!!!";
    cv::putText(original_img, message, cv::Point(0.1*original_img.cols, 0.2*original_img.rows), cv::FONT_HERSHEY_TRIPLEX, 2, cvScalar(0, 0, 0), 5, CV_FILLED);

    /*Test: Getting distance between a specified line point and the camera*/ 
    cv::Point2f stop_center_pt = cv::Point((points[4].x + points[5].x)/2, (points[4].y + points[5].y)/2);
    std::cout << "stopLine center pt: " << stop_center_pt << "\n";
    cv::circle(original_img, stop_center_pt, 6, cv::Scalar(0, 0, 255), CV_FILLED);

    float stop_dist = std::sqrt(pow(stop_center_pt.x-WIDTH/2, 2) + pow(stop_center_pt.y - original_img.rows, 2));
	std::string distance = std::to_string(stop_dist);

    cv::putText(original_img, distance, cv::Point(0.1*original_img.cols, 0.3*original_img.rows), cv::FONT_HERSHEY_TRIPLEX, 1, cvScalar(255, 0, 0), 5);
}

void ip_process(struct ip_data *ip, struct ip_res *res) {
    res->error_valid = false;
    res->stopline_found = false;

    typedef std::chrono::high_resolution_clock Clock;
	auto start = Clock::now();

    cv::Mat frame;
	ip->cap->read(frame);

	if (frame.empty()) {
	    std::cout << "Error: Empty frame\n";
	    return;
	}

    cv::Mat thres_img = threshold(frame);
    cv::Mat edges_image = img_edge_detector(thres_img);
    cv::Mat masked_image = mask_image(edges_image);

    std::vector<cv::Vec4i> hough_lines = find_lines(masked_image);
    
    if (!hough_lines.empty()) {
        std::vector<std::vector<cv::Vec4i>> lines;
        lines = classify_lines(hough_lines, edges_image);

        bool rline_found = !lines[0].empty();
        bool lline_found = !lines[1].empty();
        bool sline_found = !lines[2].empty();
        res->error_valid = lline_found && rline_found;

        std::vector<cv::Point> lane;
        lane = linear_regression(lines, frame, rline_found,
                                 lline_found, sline_found);

        int left_top = lane[3].x;
        int right_top = lane[1].x;
        res->error = ((float)(left_top+right_top)/2 - WIDTH/2)/((float)WIDTH/2);
        printf("right_top.y: %d, lefty: %d, error: %f\n", lane[1].y, lane[3].y, res->error);

#ifdef VISUAL
        plotLane(frame, lane, rline_found, lline_found, sline_found);
#endif
    }

    auto stop = Clock::now();
    double period = (double)(stop-start).count()/(1000000000);
    printf("\nFPS: %.1f\n", 1/period);

#ifdef VISUAL
    cv::createTrackbar("b,bi,tr,z,zi,msk,otsu,tri", "", &thresh_type,
                       TRACK_MAX_THRESH, NULL);
    cv::createTrackbar("thresval", "", &thresh_value, TRACK_MAX, NULL);
    cv::createTrackbar("hougthres", "", &hough_threshold, TRACK_MAX, NULL);
    cv::createTrackbar("houghmin", "", &line_min_length, TRACK_MAX, NULL);
    cv::createTrackbar("houghgap", "", &line_max_gap, TRACK_MAX, NULL);
    cv::createTrackbar("mask_start_y", "", &mask_start_y, HEIGHT, NULL);
    cv::createTrackbar("mask_end", "", &mask_end_y, HEIGHT, NULL);
    cv::createTrackbar("mask_width_top", "", &mask_width_top, WIDTH, NULL);
    cv::createTrackbar("mask_width_bot", "", &mask_width_bot, WIDTH, NULL);

    cv::Mat lines_img(cv::Mat::zeros(frame.size(), frame.type()));
    for (auto l : hough_lines) {
        cv::Point start(l[0], l[1]);
        cv::Point end(l[2], l[3]);
        cv::line(lines_img, start, end, cv::Scalar(255, 255, 255), 5, CV_AA);
    }

    cv::imshow("threshold", thres_img);
    cv::imshow("CannyEdges: ", edges_image);
    cv::imshow("mask", masked_image);
    cv::imshow("lines", lines_img);
    cv::imshow("Lane", frame);

    int k = cv::waitKey(1);
    if (k == 27)
        exit(0);
#endif

    frame.release();
}
