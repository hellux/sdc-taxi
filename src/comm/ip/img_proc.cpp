#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>
#include <cmath>

using namespace cv;

extern "C" void ip_init(void);
extern "C" struct ip_res *ip_process(void);

void ip_init(void) {

}

double img_center_pt;
bool lline_found = false;
bool rline_found = false;
//bool sline_found = false;
double rline_slope;  // y = m*x + b
double lline_slope;
double sline_slope;
cv::Point raxis_intersection; // y= m*x + b
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
    
    //cv::cornerHarris(edge_img, edge_img, 2, 3, 0.04);
    /*cv::Point2f perspectiveSrc[] = { cv::Point2f(565,470), cv::Point2f(721,470), cv::Point2f(277,698), cv::Point2f(1142,698) };
    cv::Point2f perspectiveDst[] = { cv::Point2f(300,0), cv::Point2f(980,0), cv::Point2f(300,720), cv::Point2f(980,720) };
    cv::getPerspectiveTransform(perspectiveSrc, perspectiveDst);
    cv::Mat perspectiveMatrix;*/
    
    cv::imshow("canny:", edge_img);
    
    int thresh_value = 140; /*140*/
    int max_binary_value =230; /*230*/

    //Segmentation. "Assign a label to every pixel in an image such that pixels with the
    // same label share certain characteristics.
    //Threshold: simplest and not expensive
    // Otsu better than binary but maybe much more expensive
    cv::threshold(edge_img, edge_img, thresh_value, max_binary_value, cv::THRESH_OTSU);
    cv::imshow("threshold", edge_img);
    
    float data[3] = { -1, 0, 1};
    cv::Mat kernel(1, 3, CV_32F, data);
    
    //recommended values when using filter2D
    cv::Point anchor  = cv::Point(-1, -1);
    const int DELTA = 0;
    const int DDEPTH = -1;

    //cv::warpPerspective(denoised_image, denoised_image, kernel, denoised_image.size());
    //cv::inRange(denoised_image, cv::Scalar(1), cv::Scalar(255), denoised_image);

    /*
    	cv::Canny(edge_img, edge_img, 100, 150, 3);
	Supposedly, a more simple alternative for filter2D. But it has not shown improvement
    */
    cv::filter2D(edge_img, edge_img, DDEPTH, kernel, anchor, DELTA, cv::BORDER_DEFAULT);
    cv::imshow("2Dfiltering: ", edge_img);

    return edge_img;
}

cv::Mat mask_image(cv::Mat& image) {
    cv::Mat mask(cv::Mat::zeros(image.size(), image.type()));
    
    const int pts_amount = 4;
    cv::Point p1(50, 480), p2(250, 250), p3(450, 250), p4(590, 480); //640x480
    cv::Point ROI[4] = {p1, p2, p3, p4};
    
    cv::fillConvexPoly(mask, ROI, pts_amount, cv::Scalar(255, 0, 0));
    cv::bitwise_and(image, mask, image);
    return image;
}

std::vector<cv::Vec4i> find_lines(cv::Mat& image) {
    std::vector<cv::Vec4i> lines; //Vec<int,4>

    double rho = 1;
    double theta = CV_PI / 180;
    int threshold = 75; //20
    double minLineLength = 20;//20
    double maxLineGap = 100; //30

    cv::HoughLinesP(image, lines, rho, theta, threshold, minLineLength, maxLineGap);

    return lines;
}

std::vector<std::vector<cv::Vec4i> > classify_lines(std::vector<cv::Vec4i>& lines, cv::Mat& image) {
    std::vector<std::vector<cv::Vec4i> > classified_lines(2); //3 when stop
    cv::Point start;
    cv::Point end;
    std::vector<double> slopes;
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

        if (slopes[x] > 0 && end.x > img_center_pt && start.x > img_center_pt) {
            right_lines.push_back(lines[x]);
            rline_found = true;
        }
        else if (slopes[x] < 0 && end.x < img_center_pt && start.x < img_center_pt) {
            left_lines.push_back(lines[x]);
            lline_found = true;
        }
        //else if (slopes[x] == 0) {
        //	stop_lines.push_back(lines[x]);
        //	stop_found = true;
        //	std::cout << "stop linessss: \n";
        //}
        x++;
    }

    classified_lines[0] = right_lines;
    classified_lines[1] = left_lines;
    //classified_lines[2] = stop_lines;

    return classified_lines;
}

std::vector<cv::Point> linear_regression(std::vector<std::vector<cv::Vec4i>>& lines, cv::Mat& image) {
    std::vector<cv::Point> points(4); //6 when stop
    cv::Point start;
    cv::Point end;
    cv::Vec4d right_line;
    cv::Vec4d left_line;
    //cv::Vec4d stop_line;
    std::vector<cv::Point> right_pts;
    std::vector<cv::Point> left_pts;
    //std::vector<cv::Point> stop_points;

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
            std::cout << "left_ " << start << "-" << end << "\n";
        }
        if (left_pts.size() > 0) {
            cv::fitLine(left_pts, left_line, CV_DIST_L2, 0, 0.01, 0.01);
            lline_slope = left_line[1] / left_line[0];
            laxis_intersection = cv::Point(left_line[2], left_line[3]);
        }
    }
    /*if (stop_found) {
        for (auto j : lines[2]) {
            start = cv::Point(j[0], j[1]);
            end = cv::Point(j[2], j[3]);

            stop_points.push_back(start);
            stop_points.push_back(end);
        }

        if (stop_points.size() > 0) {
            cv::fitLine(stop_points, stop_line, CV_DIST_L2, 0, 0.01, 0.01);
            sline_slope = stop_line[1] / stop_line[0];
            saxis_intersection = cv::Point(stop_line[2], stop_line[3]);
        }
    }*/

    int start_y = image.rows;
    int end_y = 200; //470

    double right_start_x = ((start_y - raxis_intersection.y) / rline_slope) + raxis_intersection.x;
    double right_end_x = ((end_y - raxis_intersection.y) / rline_slope) + raxis_intersection.x;

    double left_start_x = ((start_y - laxis_intersection.y) / lline_slope) + laxis_intersection.x;
    double left_end_x = ((end_y - laxis_intersection.y) / lline_slope) + laxis_intersection.x;

    /*double stop_start_x = ((start_y - saxis_intersection.y) / sline_slope) + saxis_intersection.x;
    double stop_end_x = ((end_y - saxis_intersection.y) / sline_slope) + saxis_intersection.x;*/

    points[0] = cv::Point(right_start_x, start_y);
    points[1] = cv::Point(right_end_x, end_y);
    points[2] = cv::Point(left_start_x, start_y);
    points[3] = cv::Point(left_end_x, end_y);
    //points[4] = cv::Point(stop_start_x, start_y);
    //points[5] = cv::Point(stop_end_x, end_y);
    
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

    cv::line(original_img, points[0], points[1], cv::Scalar(0, 255, 0), 5, CV_AA);
    cv::line(original_img, points[2], points[3], cv::Scalar(0, 255, 0), 5, CV_AA);
    cv::circle(original_img, points[0], 6, cv::Scalar(0, 0, 0), CV_FILLED);
    cv::circle(original_img, points[1], 6, cv::Scalar(0, 0, 0), CV_FILLED);
    cv::circle(original_img, points[2], 6, cv::Scalar(0, 0, 0), CV_FILLED);
    cv::circle(original_img, points[3], 6, cv::Scalar(0, 0, 0), CV_FILLED);

    rline_found = false;
    lline_found = false;

    /*if (is_stop_line) {
        cv::line(original_frame, lines[4], lines[5], cv::Scalar(255, 0, 0), 5, CV_AA);
        is_stop_line = false;
    }*/

    std::string message = "De baxar dina byxor!!!";
    cv::putText(original_img, message, cv::Point(50, 150), cv::FONT_HERSHEY_TRIPLEX, 2, cvScalar(255, 0, 0), 5, CV_FILLED);

    std::cout << "point 1: " << points[1] << "\n";
    std::cout << "center: " << img_center_pt << "\n";
    double dist;
    dist = std::sqrt(pow(points[1].x-img_center_pt, 2) + pow(points[1].y-720, 2));
    std::string distance = std::to_string(dist);

    cv::putText(original_img, distance, cv::Point(620, 500), cv::FONT_HERSHEY_TRIPLEX, 1, cvScalar(255, 0, 0), 5);
}

struct ip_res *ip_process(void) {
    std::cout << "hej från c++" << std::endl;

    /*
    cv::VideoCapture cap("road_car_view.mp4"); //road_car_view
    if (!cap.isOpened())
        return;
    */
    //cv::Mat frame = cv::imread("paso_peatonal.jpg");
    cv::VideoCapture cap(-1);
    if (!cap.isOpened()) {
        std::cout << "Hej Dennis! hitta kameran. Nununununu\n";
        return NULL;
    }
    
    std::cout << "Width 1:" << cap.get(CV_CAP_PROP_FRAME_WIDTH)<< "\n";
    std::cout << "Height 1:" << cap.get(CV_CAP_PROP_FRAME_HEIGHT)<< "\n";
    
    cap.set(CV_CAP_PROP_FRAME_WIDTH, 160);
    cap.set(CV_CAP_PROP_FRAME_HEIGHT,120);
    
    std::cout << "Set Width2: 160" << "\n";
    std::cout << "Set Height2: 120" << "\n";

    //std::cout << "FPS: " << cap.get(CV_CAP_PROP_FPS) << "\n";
    //cap.set(CV_CAP_PROP_FPS, 60);
    //std::cout << "FPS2: " << cap.get(CV_CAP_PROP_FPS) << "\n";

    std::cout << "Width 2:" << cap.get(CV_CAP_PROP_FRAME_WIDTH)<< "\n";
    std::cout << "Height 2:" << cap.get(CV_CAP_PROP_FRAME_HEIGHT)<< "\n";
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
	    std::cout << "EmptyFrame \n";
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

        } else {

            cv::imshow("Lane", frame);
            int k = cv::waitKey(25);

            if (k == 27) {
                cv::destroyAllWindows();
                break;
            }
        }
    }
    return NULL;
}
