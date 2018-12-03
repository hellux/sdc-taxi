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

static int thresh_value;
static int thresh_type;
static int hough_threshold;
static int line_min_length;
static int line_max_gap;
static int mask_width_top;
static int mask_start_y;
static int mask_end_y;
static float thresh_slope_line;
static float thresh_slope_stop;

static double weight_lw;
static double weight_lp_double;
static double weight_lp_single;
static double weight_sd;
static int thresh_lane_vis;
static int thresh_stop_vis;
static int stop_dmax;

extern "C" struct ip *ip_init();
extern "C" void ip_destroy(struct ip *ip);
extern "C" void ip_process(struct ip *ip, struct ip_res *res);

struct ip {
    cv::VideoCapture *cap;

    int lane_x;
    int lane_y;
    int lane_width;
    int lane_vis;

    int stop_x;
    int stop_y;
    int stop_diff;
    int stop_vis;
};

struct ip *ip_init() {
    struct ip *ip = (struct ip*)malloc(sizeof(*ip));

    ip->cap = new cv::VideoCapture(-1);

    if (!ip->cap->isOpened()) {
        std::cout << "Error: Camera not found\n";
        ip_destroy(ip);
        return NULL;
    }
  
    ip->cap->set(CV_CAP_PROP_FRAME_WIDTH, WIDTH);
    ip->cap->set(CV_CAP_PROP_FRAME_HEIGHT, HEIGHT);
    WIDTH = ip->cap->get(CV_CAP_PROP_FRAME_WIDTH);
    HEIGHT = ip->cap->get(CV_CAP_PROP_FRAME_HEIGHT);

    printf("ip camera at %dx%d\n", WIDTH, HEIGHT);

    thresh_value = 20;
    thresh_type = 6;
    hough_threshold = 7;
    line_min_length = 0;
    line_max_gap = 40;
    mask_width_top = 0.23*WIDTH;
    mask_start_y = 0.9*HEIGHT;
    mask_end_y = 0.52*HEIGHT;
    thresh_slope_line = 0.9;
    thresh_slope_stop = 0.8;
    stop_dmax = 0.2*HEIGHT;

    weight_lw = 0.06;
    weight_lp_double = 0.5;
    weight_lp_single = 0.3;
    weight_sd = 0.6;
    thresh_lane_vis = 5;
    thresh_stop_vis = 3;

    ip->lane_width = 0.8*WIDTH;
    ip->lane_x = WIDTH/2;
    ip->lane_y = 0.8*HEIGHT;
    ip->lane_vis = 0;

    ip->stop_x = WIDTH/2;
    ip->stop_y = mask_width_top;
    ip->stop_diff = 0;
    ip->stop_vis = 0;

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
    float bot_lx = 0;
    float bot_rx = image.rows;

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
                    lines_t& stop_lines, lines_t& rem_lines,
                    int lane_x, int lane_y,
                    int stop_x, int stop_y) {
    double lane_slope = (double)(stop_y-lane_y)/(stop_x-lane_x);
    double stop_slope = -1/lane_slope;
    double stop_m = stop_y - stop_x*stop_slope;
    for (auto line : lines) {
        cv::Point s(line[0], line[1]);
        cv::Point e(line[2], line[3]);
        double slope = (double)(e.y-s.y) / (e.x-s.x);
        if (std::abs(slope) < thresh_slope_stop) {
            if (std::abs(s.y-(stop_slope*s.x+stop_m))
             && std::abs(e.y-(stop_slope*e.x+stop_m))) {
                stop_lines.push_back(line);
            }
        } else if (std::abs(slope-lane_slope) > thresh_slope_line &&
                   e.x > lane_x && s.x > lane_x) {
            right_lines.push_back(line);
        } else if (std::abs(slope-lane_slope) > thresh_slope_line &&
                   e.x < lane_x && s.x < lane_x) {
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
        cv::line(img, start, end, cv::Scalar(100, 100, 100), 1, CV_AA);
    }
    for (auto l : right) {
        cv::Point start(l[0], l[1]);
        cv::Point end(l[2], l[3]);
        cv::line(img, start, end, cv::Scalar(255, 0, 0), 1, CV_AA);
    }
    for (auto l : left) {
        cv::Point start(l[0], l[1]);
        cv::Point end(l[2], l[3]);
        cv::line(img, start, end, cv::Scalar(0, 255, 0), 1, CV_AA);
    }
    for (auto l : stop) {
        cv::Point start(l[0], l[1]);
        cv::Point end(l[2], l[3]);
        cv::line(img, start, end, cv::Scalar(0, 0, 255), 1, CV_AA);
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

template<class T>
T median(std::vector<T> v) {
    int middle = v.size()/2;
    std::nth_element(v.begin(), v.begin()+middle, v.end());
    T median;
    if (v.size() % 2 == 0) {
        std::nth_element(v.begin(), v.begin()+middle+1, v.end());
        median = (v[middle-1]+v[middle])/2;
    } else {
        median = v[middle];
    }
    return median;
}

/* median x position of lines at y=height */
int line_pos_x(lines_t lines, int height) {
    if (lines.empty()) {
        return -1;
    } else {
        std::vector<int> pos;
        for (auto l : lines) {
            cv::Point s(l[0], l[1]), e(l[2], l[3]);
            if (s.x == e.x) {
                pos.push_back(s.x);
            } else if (s.y != e.y) {
                double k = (double)(e.y-s.y) / (e.x-s.x);
                double m = s.y-k*s.x;
                pos.push_back((int)((height-m)/k));
            }
        }
        return pos.empty() ? -1 : median(pos);
    }
}


/* median y position of lines at x=width */
int line_pos_y(lines_t lines, int width) {
    if (lines.empty()) {
        return -1;
    } else {
        std::vector<int> pos;
        for (auto l : lines) {
            cv::Point s(l[0], l[1]), e(l[2], l[3]);
            if (s.y == e.y) {
                pos.push_back(s.y);
            } else if (s.x != e.x) {
                double k = (double)(e.x-s.x) / (e.y-s.y);
                double m = s.x-k*s.y;
                pos.push_back((int)((width-m)/k));
            }
        }
        return pos.empty() ? -1 : median(pos);
    }
}

void ip_process(struct ip *ip, struct ip_res *res) {
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
                   right, left, stop, rem,
                   ip->lane_x, ip->lane_y,
                   ip->stop_x, ip->stop_y);

    /* calculate lane position */
    int right_pos = line_pos_x(right, ip->lane_y);
    int left_pos = line_pos_x(left, ip->lane_y);

    if (right_pos != -1 && left_pos != -1) {
        int lw = right_pos - left_pos;
        ip->lane_width = (ip->lane_width +lw*weight_lw)/(1.0+weight_lw);
        ip->lane_vis++;
    } else {
        ip->lane_vis--;
    }

    ip->lane_vis = std::min(std::max(ip->lane_vis, 0), thresh_lane_vis*2);
    
    double weight_lp = 0;
    int lp = 0;

    if (right_pos != -1 && left_pos != -1) {
        lp = (right_pos + left_pos)/2;
        weight_lp = weight_lp_double;
    } else if (right_pos != -1) {
        lp = right_pos - ip->lane_width/2;
        weight_lp = weight_lp_single;
    } else if (left_pos != -1) {
        lp = left_pos + ip->lane_width/2;
        weight_lp = weight_lp_single;
    }

    ip->lane_x = (ip->lane_x+lp*weight_lp)/(1.0+weight_lp);

    /* calc stopline position */

    double stop_right = line_pos_x(right, ip->stop_y);
    double stop_left = line_pos_x(left, ip->stop_y);
    ip->stop_x = (stop_right+stop_left)/2;

    int stop_y = line_pos_y(stop, ip->stop_x);

    if (stop_y != -1) {
        ip->stop_vis++;
        int diff = std::max(ip->stop_y-stop_y, 0);
        ip->stop_diff = (ip->stop_diff+diff*weight_sd)/(1+weight_sd);
        ip->stop_y = stop_y;
    } else {
        ip->stop_vis--;
        ip->stop_y = ip->stop_y + ip->stop_diff;
    }

    if (ip->stop_y < mask_end_y || HEIGHT <= ip->stop_y) {
        ip->stop_y = mask_end_y;
        ip->stop_diff = 0;
        ip->stop_vis = 0;
    }

    ip->stop_vis = std::min(std::max(ip->stop_vis, 0), thresh_stop_vis*2);

    /* write to result struct */
    res->lane_offset = (float)ip->lane_x/(WIDTH/2) - 1;
    res->lane_found = ip->lane_vis >= thresh_lane_vis;
    res->stopline_dist = 1-(float)ip->stop_y/HEIGHT; /* TODO calc in meters */
    res->stopline_found = ip->stop_vis >= thresh_stop_vis;

#ifdef VISUAL
    if (!hough_lines.empty()) {
        std::vector<cv::Point> lane;
        lane = linear_regression(right, left, stop);
        plotLane(frame, lane);
    }

    auto stop_time = Clock::now();
    double period = (double)(stop_time-start).count()/(1e9);
	std::string fps = std::to_string((int)(1/period));
    std::string err = std::to_string(res->lane_offset);
    cv::putText(frame, fps, cv::Point(0,15), FONT, 1, cvScalar(0,0,0), 2);
    cv::putText(frame, err, cv::Point(0,30), FONT, 1, cvScalar(0,100,0), 2);

    cv::createTrackbar("b,bi,tr,z,zi,msk,otsu,tri", "", &thresh_type,
                       TRACK_MAX_THRESH, NULL);
    cv::createTrackbar("thresval", "", &thresh_value, TRACK_MAX, NULL);
    cv::createTrackbar("hougthres", "", &hough_threshold, TRACK_MAX, NULL);
    cv::createTrackbar("houghmin", "", &line_min_length, TRACK_MAX, NULL);
    cv::createTrackbar("houghgap", "", &line_max_gap, TRACK_MAX, NULL);
    cv::createTrackbar("mask_width_top", "", &mask_width_top, WIDTH, NULL);
    cv::createTrackbar("mask_start_y", "", &mask_start_y, HEIGHT, NULL);
    cv::createTrackbar("mask_end_y", "", &mask_end_y, HEIGHT, NULL);

    cv::Mat lines_img(cv::Mat::zeros(frame.size(), frame.type()));
    plot_lines(lines_img, right, left, stop, rem);
    cv::line(lines_img,
             cv::Point(ip->lane_x - ip->lane_width/2, ip->lane_y),
             cv::Point(ip->lane_x + ip->lane_width/2, ip->lane_y),
             cv::Scalar(255,255,0), 1, CV_AA);
    int lane_thick = res->lane_found ? 3 : 1;
    cv::line(lines_img,
             cv::Point(ip->lane_x, ip->lane_y),
             cv::Point(ip->stop_x, ip->stop_y),
             cv::Scalar(0,255,255), lane_thick, CV_AA);
    int stop_thick = res->stopline_found ? 3 : 1;
    cv::line(lines_img,
             cv::Point(0, ip->stop_y),
             cv::Point(WIDTH, ip->stop_y),
             cv::Scalar(255,0,255), stop_thick, CV_AA);
    cv::circle(lines_img,
             cv::Point(left_pos, ip->lane_y), 3,
             cv::Scalar(0,255,0), CV_FILLED);
    cv::circle(lines_img,
             cv::Point(right_pos, ip->lane_y), 3,
             cv::Scalar(255,0,0), CV_FILLED);

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
