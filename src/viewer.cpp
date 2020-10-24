//
// Created by gaoxiang on 19-5-4.
//
#include "myslam/viewer.h"
#include "myslam/feature.h"
#include "myslam/frame.h"

#include <pangolin/pangolin.h>
#include <opencv2/opencv.hpp>

namespace myslam {

Viewer::Viewer() {
    // viewer_thread_ = std::thread(std::bind(&Viewer::ThreadLoop, this));//初始化线程
}

void Viewer::Close() {//ThreadLoop 结束
    viewer_running_ = false;
    viewer_thread_.join();
}

/*
AddCurrentFrame, UpdateMap 都是在更新可视化所需的数据(写操作)
ThreadLoop 读取数据进行可视化(pangolin, opencv)（读操作）

*/
void Viewer::AddCurrentFrame(Frame::Ptr current_frame) {
    std::unique_lock<std::mutex> lck(viewer_data_mutex_); // unique_lock<mutex> 会自动对互斥量上锁，生命周期结束后会自动释放锁
    current_frame_ = current_frame;// 更新 current_frame_  (写操作)
}

void Viewer::UpdateMap() {
    std::unique_lock<std::mutex> lck(viewer_data_mutex_);
    assert(map_ != nullptr);
    active_keyframes_ = map_->GetActiveKeyFrames();// 更新　active_keyframes, active_landmarks （写操作）
    active_landmarks_ = map_->GetActiveMapPoints();
    map_updated_ = true;
}

void Viewer::ThreadLoop() {// 可视化线程
    /*
    1.创建pangolin窗口
    2. 设置相机投影矩阵和观测点
    3. 绘图(读取 current_frame_, active_keyframes, active_landmarks   保证对这些数据互斥即可)
        DrawFrame   需要数据: current_frame_
        FollowCurrentFrame 需要:current_frame_提供的Twc
        PlotFrameImage : 需要: current_frame cv 产生的场景窗口, 展示特征点
        DrawMapPoints:  需要 active_keyframes, active_landmarks
    */
    pangolin::CreateWindowAndBind("MySLAM", 1024, 768);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);




    pangolin::OpenGlRenderState vis_camera(
        pangolin::ProjectionMatrix(1024, 768, 400, 400, 512, 384, 0.1, 1000),
        pangolin::ModelViewLookAt(0, -5, -10, 0, 0, 0, 0.0, -1.0, 0.0));

    // Add named OpenGL viewport to window and provide 3D Handler
    pangolin::View& vis_display =
        pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, 0.0, 1.0, -1024.0f / 768.0f)
            .SetHandler(new pangolin::Handler3D(vis_camera));

    const float blue[3] = {0, 0, 1};
    const float green[3] = {0, 1, 0};

    while (!pangolin::ShouldQuit() && viewer_running_) {//pangolin 和　Opencv 的可视化
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
        vis_display.Activate(vis_camera);

        std::unique_lock<std::mutex> lock(viewer_data_mutex_);
        if (current_frame_) {
            DrawFrame(current_frame_, green);//画当前帧（绿色）
            FollowCurrentFrame(vis_camera);

            cv::Mat img = PlotFrameImage();
            cv::imshow("image", img);
            cv::waitKey(1);
        }

        if (map_) {
            DrawMapPoints();//画激活帧和landmarks(红色)
        }

        pangolin::FinishFrame();
        usleep(5000);
    }

    LOG(INFO) << "Stop viewer";
}

cv::Mat Viewer::PlotFrameImage() {
    cv::Mat img_out;
    cv::cvtColor(current_frame_->left_img_, img_out, CV_GRAY2BGR);
    for (size_t i = 0; i < current_frame_->features_left_.size(); ++i) {
        if (current_frame_->features_left_[i]->map_point_.lock()) {
            auto feat = current_frame_->features_left_[i];
            cv::circle(img_out, feat->position_.pt, 2, cv::Scalar(0, 250, 0),
                       2);
        }
    }
    return img_out;
}

void Viewer::FollowCurrentFrame(pangolin::OpenGlRenderState& vis_camera) {
    SE3 Twc = current_frame_->Pose().inverse();
    pangolin::OpenGlMatrix m(Twc.matrix());
    vis_camera.Follow(m, true);//视角会跟随当前帧
}

void Viewer::DrawFrame(Frame::Ptr frame, const float* color) {
    SE3 Twc = frame->Pose().inverse();
    const float sz = 1.0;
    const int line_width = 2.0;
    const float fx = 400;
    const float fy = 400;
    const float cx = 512;
    const float cy = 384;
    const float width = 1080;
    const float height = 768;

    glPushMatrix();

    Sophus::Matrix4f m = Twc.matrix().template cast<float>();
    glMultMatrixf((GLfloat*)m.data());

    if (color == nullptr) {
        glColor3f(1, 0, 0);
    } else
        glColor3f(color[0], color[1], color[2]);

    glLineWidth(line_width);
    glBegin(GL_LINES);
    glVertex3f(0, 0, 0);//像素坐标系(width*height)回到相机坐标系，深度为sz
    glVertex3f(sz * (0 - cx) / fx, sz * (0 - cy) / fy, sz);
    glVertex3f(0, 0, 0);
    glVertex3f(sz * (0 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
    glVertex3f(0, 0, 0);
    glVertex3f(sz * (width - 1 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
    glVertex3f(0, 0, 0);
    glVertex3f(sz * (width - 1 - cx) / fx, sz * (0 - cy) / fy, sz);

    glVertex3f(sz * (width - 1 - cx) / fx, sz * (0 - cy) / fy, sz);
    glVertex3f(sz * (width - 1 - cx) / fx, sz * (height - 1 - cy) / fy, sz);

    glVertex3f(sz * (width - 1 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
    glVertex3f(sz * (0 - cx) / fx, sz * (height - 1 - cy) / fy, sz);

    glVertex3f(sz * (0 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
    glVertex3f(sz * (0 - cx) / fx, sz * (0 - cy) / fy, sz);

    glVertex3f(sz * (0 - cx) / fx, sz * (0 - cy) / fy, sz);
    glVertex3f(sz * (width - 1 - cx) / fx, sz * (0 - cy) / fy, sz);

    glEnd();
    glPopMatrix();
}

void Viewer::DrawMapPoints() { 
    const float red[3] = {1.0, 0, 0};
    for (auto& kf : active_keyframes_) {//画激活帧
        DrawFrame(kf.second, red);
    }

    glPointSize(2);
    glBegin(GL_POINTS);
    for (auto& landmark : active_landmarks_) {//画激活landmarks
        auto pos = landmark.second->Pos();
        glColor3f(red[0], red[1], red[2]);
        glVertex3d(pos[0], pos[1], pos[2]);
    }
    glEnd();
}

}  // namespace myslam
