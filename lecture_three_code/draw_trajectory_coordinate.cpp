//
// Created by weihao on 2022/3/13.
//
//
// Created by wenbo on 2020/10/23.
//
#include <pangolin/pangolin.h>
#include <Eigen/Core>
#include <unistd.h>

// 本例演示了如何画出一个预先存储的轨迹

using namespace std;
using namespace Eigen;
string trajectory_file = "../trajectory.txt";

//定义一个画轨迹的函数
void DrawTrajectory(vector<Isometry3d, Eigen::aligned_allocator<Isometry3d>>);

int main() {
    vector<Isometry3d, Eigen::aligned_allocator<Isometry3d>> poses;// 定义容器  装poses（r到world坐标的变换矩阵）
    //  Eigen::aligned_allocator<Isometry3d> Eigen管理内存和C++11中的方法是不一样的，所以需要单独强调元素的内存分配和管理
    ifstream fin(trajectory_file);

    if (!fin) {
        cout << "cannot find trajectory file at" << "trajectory_file" << endl;
    }
    while (!fin.eof()) {
        double time, tx, ty, tz, qx, qy, qz, qw;//定义获取文件中平移和四元数（旋转）的变量
        fin >> time >> tx >> ty >> tz >> qx >> qy >> qz >> qw;
        Isometry3d Twr(Quaterniond(qw, qx, qy, qz));
        Twr.pretranslate((Vector3d(tx, ty, tz)));
        poses.push_back(Twr);
    }
    cout << "read total " << poses.size() << " poses entries " << endl;
    //调用DrawTrajectory()函数
    DrawTrajectory(poses);
    return 0;
}

void DrawTrajectory(vector<Isometry3d, Eigen::aligned_allocator<Isometry3d>> poses) {
    // 创建 pangolin 窗口，画轨迹
    //creat and set titles 、 width 、height of windows
    pangolin::CreateWindowAndBind(" Trajectory ", 1024, 768);
    //glEnable(GL_DEPTH_TEST)启用了之后，OpenGL在绘制的时候就会检查，当前像素前面是否有别的像素，如果别的像素挡道了它，那它就不会绘制，也就是说，OpenGL就只绘制最前面的一层。
    //当我们需要绘制透明图片时，就需要关闭它glDisable(GL_DEPTH_TEST);
    //并且打开混合 glEnable(GL_BLEND);
    glEnable(GL_DEPTH_TEST);//??
    glEnable(GL_BLEND);//??
    //glBlendFunc(GLenum sfactor,GLenum dfactor);
    //源因子和目标因子是可以通过glBlendFunc函数来进行设置的。
    // glBlendFunc有两个参数，前者sfactor表示源因子，后者dfactor表示目标因子。
    //前者sfactor表示源颜色，后者dfactor表示目标颜色
    //GL_SRC_ALPHA：表示使用源颜色的alpha值来作为因子
    //GL_ONE_MINUS_SRC_ALPHA：表示用1.0减去源颜色的alpha值来作为因子（1-alpha)
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    //OpenGlRenderState 创建一个相机的观察视图，模拟相机
    pangolin::OpenGlRenderState s_cam(//定义投影和初始模型视图矩阵
            //ProjectionMatrix 前两个参数是相机的宽高，紧接着四个参数是相机的内参，最后两个是最近和最远视距
            pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
            //ModelViewLookAt 前三个参数是相机的位置，紧接着三个是相机所看的视点的位置，最后三个参数是一个向量，表示相机的的朝向
            pangolin::ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, 0.0, -1.0, 0.0)
    );
    //在窗口创建交互式视图
    pangolin::View &d_cam = pangolin::CreateDisplay().SetBounds(0.0, 1.0, 0.0, 1.0, -1024.0f / 768.0f).
            //SetBounds() 前四个参数是视图在视窗中的范围（下 上 左 右）
            SetHandler(new pangolin::Handler3D(s_cam));
    //     SetHandle设置相机的视图句柄，需要用它来显示前面设置的 “相机” 所 “拍摄” 的内容
    while (pangolin::ShouldQuit() == false) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);//清除屏幕（颜色缓冲、深度缓冲）
        d_cam.Activate(s_cam);//激活显示，并设置状态矩阵
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
        glLineWidth(2);//线宽
        for (size_t i = 0; i < poses.size(); i++) {
            //画出每个位子的三个坐标轴，此处的 translation 表示的应该是位置
            Vector3d Ow = poses[i].translation();
            Vector3d Xw = poses[i] * (0.1 * Vector3d(1, 0, 0));
            Vector3d Yw = poses[i] * (0.1 * Vector3d(0, 1, 0));
            Vector3d Zw = poses[i] * (0.1 * Vector3d(0, 0, 1));
            glBegin(GL_LINES);
            glColor3f(1.0, 0.0, 0.0);//设置x轴的颜色
            glVertex3d(Ow[0], Ow[1], Ow[2]);
            glVertex3d(Xw[0], Xw[1], Xw[2]);//把两个点连线，后面同理
            glColor3f(0.0, 1.0, 0.0);
            glVertex3d(Ow[0], Ow[1], Ow[2]);
            glVertex3d(Yw[0], Yw[1], Yw[2]);
            glColor3f(0.0, 0.0, 1.0);
            glVertex3d(Ow[0], Ow[1], Ow[2]);
            glVertex3d(Zw[0], Zw[1], Zw[2]);
            glEnd();
        }
        //把每一个位姿连线
        for (size_t i = 0; i < poses.size(); i++) {
            glColor3f(0.0, 0.0, 0.0);
            glBegin(GL_LINES);
            auto p1 = poses[i], p2 = poses[i + 1];
            glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
            glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
            glEnd();
        }
        pangolin::FinishFrame();
        usleep(5000);//休眠5ms
    }

}

