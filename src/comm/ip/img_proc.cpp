#include "img_proc.h"

#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>
#include <cmath>
#include <opencv2/highgui/highgui.hpp>
#include <chrono>

typedef std::vector<cv::Vec4i> lines_t;

#define FONT cv::FONT_HERSHEY_PLAIN

static int WIDTH = 256;
static int HEIGHT = 144;

const int THR_TYPES[] = {cv::THRESH_BINARY, cv::THRESH_BINARY_INV,
                         cv::THRESH_TRUNC, cv::THRESH_TOZERO,
                         cv::THRESH_TOZERO_INV, cv::THRESH_MASK,
                         cv::THRESH_OTSU};
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

static int measure_height = 0.8*HEIGHT;
static int weight_lw = 1;
static int weight_lp_double = 0.5;
static int weight_lp_single = 0.3;

extern "C" struct ip *ip_init();
extern "C" void ip_destroy(struct ip *ip);
extern "C" void ip_process(struct ip *ip, struct ip_res *res);

struct ip {
    cv::VideoCapture *cap;

    int lane_width;
    int lane_pos;
};

struct ip *ip_init() {
    struct ip *ip = (struct ip*)malloc(sizeof(*ip));

    ip->cap = new cv::VideoCapture(-1);
    ip->lane_width = 0;
    ip->lane_pos = 0;

    if (!ip->cap->isOpened()) {
        std::cout << "Error: Camera not found\n";
        ip_destroy(ip);
        return NULL;
    }
  
    ip->cap->set(CV_CAP_PROP_FRAME_WIDTH, WIDTH);
    ip->cap->set(CV_CAP_PROP_FRAME_HEIGHT, HEIGHT);
    WIDTH = ip->cap->get(CV_CAP_PROP_FRAME_WIDTH);
    HEIGHT = ip->cap->get(CV_CAP_PROP_FRAME_HEIGHT);

#ifdef VISUAL
    cv::namedWindow("Lane", CV_WINDOW_AUTOSIZE);
    cv::namedWindow("", CV_WINDOW_AUTOSIZE);
#endif

    return ip;
}

void ip_destroy(struct ip *ip) {
#ifdef VISUAL
    cv::destroyAllWindows();
#endif
    if (ip) {
        delete ip->cap;
    }
}

cv::Mat threshold(cv::Mat& image) {
    cv::Mat edge_img;
    cv::GaussianBlur(image, edge_img, cv::Size(3, 3), 0, 0);
    cv::cvtColor(edge_img, edge_img, cv::COLOR_RGB2GRAY);
    cv::threshold(edge_img, edge_img, thresh_value, 255,
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

    /* region of interest */
    float top_y = mask_end_y;
    float top_rx = (image.cols+mask_width_top)/2;
    float top_lx = (image.cols-mask_width_top)/2;

    float bot_y = mask_start_y;
    float bot_lx = (image.cols-mask_width_bot)/2;
    float bot_rx = (image.cols+mask_width_bot)/2;

    cv::Point p0(0, HEIGHT), p1(bot_lx, bot_y), p2(top_lx, top_y),
              p3(top_rx, top_y), p4(bot_rx, bot_y), p5(WIDTH, HEIGHT);
    cv::Point roi[] = {p0, p1, p2, p3, p4, p5};
    
    cv::fillConvexPoly(mask, roi, sizeof(roi)/sizeof(*roi), cv::Scalar(255, 0, 0));
    cv::bitwise_and(image, mask, masked_image);

    return masked_image;
}

lines_t find_lines(cv::Mat& image) {
    double rho = 1;
    double theta = CV_PI / 180;
    lines_t lines;
    cv::HoughLinesP(image, lines, rho, theta, hough_threshold,
                    line_min_length, line_max_gap);
    return lines;
}

void classify_lines(lines_t& lines, cv::Mat& image,
                    lines_t& right_lines, lines_t& left_lines,
                    lines_t& stop_lines, lines_t& rem_lines) {
    int cx = WIDTH/2;
    for (auto line : lines) {
        cv::Point s(line[0], line[1]);
        cv::Point e(line[2], line[3]);
        double slope = (double)(e.y-s.y) / (e.x-s.x);
        if (slope == 0 || (s.x < cx && e.x > cx)) {
            stop_lines.push_back(line);
        } else if (slope > 1 && (e.x > cx && s.x > cx && e.y > .95*HEIGHT)) {
            right_lines.push_back(line);
        } else if (slope < -1 && (e.x < cx && s.x < cx && s.y > .95*HEIGHT)) {
            left_lines.push_back(line);
        } else {
            rem_lines.push_back(line);
        }
    }
}

std::vector<cv::Point> linear_regression(lines_t& lines_right,
                                         lines_t& lines_left,
                                         lines_t& lines_stop) {
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

    if (!lines_right.empty()) {
        for (auto points : lines_right) {
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
    if (!lines_left.empty()) {
        for (auto point : lines_left) {
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
    if (!lines_stop.empty()) {
        for (auto point : lines_stop) {
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

    int lines_start_y = HEIGHT;
    int lines_end_y = 0.60*HEIGHT;
    int sline_start_x = 0.20*WIDTH;
    int sline_end_x = 0.80*WIDTH;

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

#ifdef VISUAL

void plot_lines(cv::Mat& img, lines_t right, lines_t left,
                              lines_t stop, lines_t rem) {
    for (auto l : rem) {
        cv::Point start(l[0], l[1]);
        cv::Point end(l[2], l[3]);
        cv::line(img, start, end, cv::Scalar(100, 100, 100), 2, CV_AA);
    }
    for (auto l : right) {
        cv::Point start(l[0], l[1]);
        cv::Point end(l[2], l[3]);
        cv::line(img, start, end, cv::Scalar(255, 0, 0), 2, CV_AA);
    }
    for (auto l : left) {
        cv::Point start(l[0], l[1]);
        cv::Point end(l[2], l[3]);
        cv::line(img, start, end, cv::Scalar(0, 255, 0), 2, CV_AA);
    }
    for (auto l : stop) {
        cv::Point start(l[0], l[1]);
        cv::Point end(l[2], l[3]);
        cv::line(img, start, end, cv::Scalar(0, 0, 255), 2, CV_AA);
    }
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
    cv::circle(original_img, points[0], 6, cv::Scalar(0, 0, 0), CV_FILLED);
    cv::circle(original_img, points[1], 6, cv::Scalar(0, 0, 0), CV_FILLED);

    cv::line(original_img, points[2], points[3], cv::Scalar(255, 0, 0), 5, CV_AA);
    cv::circle(original_img, points[2], 6, cv::Scalar(0, 0, 0), CV_FILLED);
    cv::circle(original_img, points[3], 6, cv::Scalar(0, 0, 0), CV_FILLED);

    cv::line(original_img, points[4], points[5], cv::Scalar(0, 0, 0), 5, CV_AA);
    cv::circle(original_img, points[4], 6, cv::Scalar(0, 0, 255), CV_FILLED);
    cv::circle(original_img, points[5], 6, cv::Scalar(0, 0, 255), CV_FILLED);
    
    /*Test: Getting distance between a specified line point and the camera*/ 
    cv::Point2f stop_center_pt = cv::Point((points[4].x + points[5].x)/2, (points[4].y + points[5].y)/2);
    cv::circle(original_img, stop_center_pt, 6, cv::Scalar(0, 0, 255), CV_FILLED);

    float stop_dist = std::sqrt(pow(stop_center_pt.x-WIDTH/2, 2) + pow(stop_center_pt.y - original_img.rows, 2));
	std::string distance = std::to_string(stop_dist);
    cv::putText(original_img, distance, cv::Point(0, 45), FONT, 1,
                cvScalar(255, 0, 0), 2);
}

#endif

/* average x position of lines at y=height */
int line_pos(struct ip *ip, lines_t lines, int height) {
    if (lines.empty()) {
        return 0;
    } else {
        double sum = 0;
        for (auto l : lines) {
            cv::Point s(l[0], l[1]), e(l[2], l[3]);
            double k = (double)(e.y-s.y) / (e.x-s.x);
            double m = s.y-k*s.x;
            sum += (height-m)/k;
        }
        return (int)(sum/lines.size());
    }
}

void ip_process(struct ip *ip, struct ip_res *res) {
    res->error_valid = false;
    res->stopline_found = false;

#ifdef VISUAL
    typedef std::chrono::high_resolution_clock Clock;
	auto start = Clock::now();
#endif

    cv::Mat frame;
	ip->cap->read(frame);

	if (frame.empty()) {
	    std::cout << "Error: Empty frame\n";
	    return;
	}

    /* process image */
    cv::Mat thres_img = threshold(frame);
    cv::Mat edges_image = img_edge_detector(thres_img);
    cv::Mat masked_image = mask_image(edges_image);

    /* find and classify lines */
    lines_t hough_lines = find_lines(masked_image);
    lines_t right, left, stop, rem;
    classify_lines(hough_lines, edges_image,
                   right, left, stop, rem);

    /* calculate error */
    int right_pos = line_pos(ip, right, measure_height);
    int left_pos = line_pos(ip, left, measure_height);

    if (!right.empty() && !left.empty()) {
        int lw = right_pos - left_pos;
        ip->lane_width = (ip->lane_width + lw*weight_lw)/(1+weight_lw);
    }
    
    double weight_lp = 0;
    int lp = 0;

    if (!right.empty() && !left.empty()) {
        lp = (right_pos + left_pos)/2;
        weight_lp = weight_lp_double;
    } else if (!right.empty()) {
        lp = right_pos - ip->lane_width/2;
        weight_lp = weight_lp_single;
    } else if (!left.empty()) {
        lp = left_pos + ip->lane_width/2;
        weight_lp = weight_lp_single;
    }

    ip->lane_pos = (ip->lane_pos + lp*weight_lp)/(1+weight_lp);

    res->error = (float)ip->lane_pos/WIDTH;

#ifdef VISUAL
    if (!hough_lines.empty()) {
        std::vector<cv::Point> lane;
        lane = linear_regression(right, left, stop);
        plotLane(frame, lane);
    }

    auto stop_time = Clock::now();
    double period = (double)(stop_time-start).count()/(1e9);
	std::string fps = std::to_string((int)(1/period));
    std::string err = std::to_string(res->error);
    cv::putText(frame, fps, cv::Point(0,15), FONT, 1, cvScalar(0,0,0), 2);
    cv::putText(frame, err, cv::Point(0,30), FONT, 1, cvScalar(0,100,0), 2);

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
    plot_lines(lines_img, right, left, stop, rem);

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
