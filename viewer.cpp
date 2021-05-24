#include "viewer.h"

#include "logger.h"


Viewer::Viewer()
    :win_width(600), win_height(400)
{
}

static void glfwErrorCallback(int error, const char* description)
{
  std::cerr << "GLFW error " << error << " " << description << std::endl;
}


static std::string readShader(const std::string &filename) {
    std::ifstream is(filename.c_str());
    if (is) {
        is.seekg(0, is.end);
        int length = is.tellg();
        is.seekg(0, is.beg);

        char *buffer = new char[length];
        is.read(buffer, length);
        if (!is){
            std::cout << "failed to read shader!" << std::endl;
        }
        std::string s(buffer);
        delete[] buffer;
        return s;
    } else {
        std::cout << "failed to read shader!" << std::endl;
        return std::string();
    }
}


void Viewer::setWindowTitle(std::string title)
{
    glfwSetWindowTitle(window, title.c_str());
}


void Viewer::initialize()
{
    // init glfw - if already initialized nothing happens
    glfwInit();

    GLFWerrorfun prev_func = glfwSetErrorCallback(glfwErrorCallback);
    if (prev_func)
      glfwSetErrorCallback(prev_func);

    // setup context
    glfwDefaultWindowHints();
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 1);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_ANY_PROFILE);

    window = glfwCreateWindow(win_width*2, win_height*2, "Viewer (press ESC to exit)", 0, NULL);
    if (window == NULL)
    {
        std::cerr << "Failed to create opengl window." << std::endl;
        exit(-1);
    }

    glfwMakeContextCurrent(window);
    OpenGLBindings *b = new OpenGLBindings();
    flextInit(b);
    gl(b);

    std::string vertexshadersrc = readShader(context->mShaderFolder + "common.vs");
    std::string grayfragmentshader = readShader(context->mShaderFolder + "gray.fs");
    std::string fragmentshader = readShader(context->mShaderFolder + "color.fs");

    renderShader.setVertexShader(vertexshadersrc);
    renderShader.setFragmentShader(fragmentshader);
    renderShader.build();

    renderGrayShader.setVertexShader(vertexshadersrc);
    renderGrayShader.setFragmentShader(grayfragmentshader);
    renderGrayShader.build();

    glfwSetWindowUserPointer(window, this);
    glfwSetKeyCallback(window, Viewer::key_callbackstatic);
    glfwSetWindowSizeCallback(window, Viewer::winsize_callbackstatic);
    glfwSetMouseButtonCallback(window, Viewer::mouse_button_callbackstatic);
}


void Viewer::winsize_callbackstatic(GLFWwindow* window, int w, int h)
{
    Viewer* viewer = reinterpret_cast<Viewer*>(glfwGetWindowUserPointer(window));
    viewer->winsize_callback(window, w, h);
}


void Viewer::winsize_callback(GLFWwindow* window, int w, int h)
{
    win_width = w/2;
    win_height = h/2;
}


void Viewer::mouse_button_callbackstatic(GLFWwindow* window, int button, int action, int mods)
{
    Viewer* viewer = reinterpret_cast<Viewer*>(glfwGetWindowUserPointer(window));
    viewer->mouse_button_callback(window, button, action, mods);
}


void Viewer::mouse_button_callback(GLFWwindow* window, int button, int action, int mods)
{
    double xpos, ypos;
    int frame_xpos, frame_ypos;
    std::string frameIdx;
    if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_PRESS) {
        glfwGetCursorPos(window, &xpos, &ypos);
        if (xpos < win_width && ypos < win_height ) {
            frameIdx = context->mKinectViewerMap[0];
            frame_xpos = xpos;
            frame_ypos = ypos;
        } else if (xpos >= win_width && ypos < win_height) {
            frameIdx = context->mKinectViewerMap[1];
            frame_xpos = xpos - win_width;
            frame_ypos = ypos;
        } else if (xpos < win_width && ypos >= win_height) {
            frameIdx = context->mKinectViewerMap[2];
            frame_xpos = xpos;
            frame_ypos = ypos - win_height;
        } else if (xpos >= win_width && ypos >= win_height) {
            frameIdx = context->mKinectViewerMap[3];
            frame_xpos = xpos - win_width;
            frame_ypos = ypos - win_height;
        }

        std::cout << "==============" << std::endl;

        std::shared_ptr<Frame> frame = frames[frameIdx];
        float xscale = (float)frame->getWidth() / win_width;
        float yscale = (float)frame->getHeight() / win_height;

        auto pixel1 = frame->getRGBPixel(frame_xpos * xscale, frame_ypos * yscale);
        std::cout << pixel1 << std::endl;

        auto pixel2 = frame->getDepthPixel(frame_xpos * xscale, frame_ypos * yscale);
        std::cout << pixel2 << std::endl;

        auto pixel3 = frame->getJointPixel(frame_xpos * xscale, frame_ypos * yscale);
        std::cout << pixel3 << std::endl;
    }
}


void Viewer::key_callbackstatic(GLFWwindow* window, int key, int scancode, int action, int mods)
{
    Viewer* viewer = reinterpret_cast<Viewer*>(glfwGetWindowUserPointer(window));
    viewer->key_callback(window, key, scancode, action, mods);
}


void Viewer::key_callback(GLFWwindow* window, int key, int scancode, int action, int mods)
{
    //EXIT
    if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS) {
        context->mExit = true;
    }
    //RECORD
    if ((key == GLFW_KEY_F1 && action == GLFW_PRESS)
        || (key == GLFW_KEY_PAGE_UP && action == GLFW_PRESS)) {
        context->mStartRecord = true;
    }
    if ((key == GLFW_KEY_F2 && action == GLFW_PRESS)
        || (key == GLFW_KEY_PAGE_DOWN && action == GLFW_PRESS)) {
        context->mStopRecord = true;
    }
    //REPLAY
    if (key == GLFW_KEY_F3 && action == GLFW_PRESS) {
        if (context->mReplay){
            context->mStopReplay = true;
        } else {
            context->mStartReplay = true;
        }
    }
    //LOAD
    if (key == GLFW_KEY_F4 && action == GLFW_PRESS) {
        if (!context->mReplay){
            context->mLoad = true;
        }
    }
    //CALIBRATE
    if (key == GLFW_KEY_F5 && action == GLFW_PRESS) {
        context->mHomography = !context->mHomography;
    }
}


void Viewer::onOpenGLBindingsChanged(OpenGLBindings *b)
{
    renderShader.gl(b);
    renderGrayShader.gl(b);
    rgb.gl(b);
    ir.gl(b);
}


bool Viewer::render()
{
    // wipe the drawing surface clear
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    GLint x = 0, y = 0;
    int fb_width, fb_width_half, fb_height, fb_height_half;

    for (auto iter = frames.begin(); iter != frames.end(); ++iter)
    {
        const auto &frame = iter->second;

        // Using the frame buffer size to account for screens where window.size != framebuffer.size, e.g. retina displays
        glfwGetFramebufferSize(window, &fb_width, &fb_height);
        fb_width_half = (fb_width + 1) / 2;
        fb_height_half = (fb_height + 1) / 2;

        glViewport(x, y, fb_width_half, fb_height_half);
        x += fb_width_half;
        if (x >= (fb_width - 1))
        {
            x = 0;
            y += fb_height_half;
        }

        float w = (float)frame->getWidth();
        float h = (float)frame->getHeight();

        Vertex bl = { -1.0f, -1.0f, 0.0f, 0.0f };
        Vertex br = { 1.0f, -1.0f, w, 0.0f }; 
        Vertex tl = { -1.0f, 1.0f, 0.0f, h };
        Vertex tr = { 1.0f, 1.0f, w, h };
        Vertex vertices[] = {
            bl, tl, tr, 
            tr, br, bl
        };

        gl()->glGenBuffers(1, &triangle_vbo);
        gl()->glGenVertexArrays(1, &triangle_vao);

        gl()->glBindVertexArray(triangle_vao);
        gl()->glBindBuffer(GL_ARRAY_BUFFER, triangle_vbo);
        gl()->glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);

        GLint position_attr = renderShader.getAttributeLocation("Position");
        gl()->glVertexAttribPointer(position_attr, 2, GL_FLOAT, GL_FALSE, sizeof(Vertex), (GLvoid*)0);
        gl()->glEnableVertexAttribArray(position_attr);

        GLint texcoord_attr = renderShader.getAttributeLocation("TexCoord");
        gl()->glVertexAttribPointer(texcoord_attr, 2, GL_FLOAT, GL_FALSE, sizeof(Vertex), (GLvoid*)(2 * sizeof(float)));
        gl()->glEnableVertexAttribArray(texcoord_attr);


        if (frame->getType() == openni::SensorType::SENSOR_IR)
        {
            renderShader.use();

            rgb.allocate(frame->getWidth(), frame->getHeight());
            memcpy(rgb.data, frame->getData(), frame->getDataSize());
            rgb.flipY();
            rgb.upload();
            glDrawArrays(GL_TRIANGLES, 0, 6);
            rgb.deallocate();
        }
        if (frame->getType() == openni::SensorType::SENSOR_COLOR)
        {
            renderShader.use();

            rgb.allocate(frame->getWidth(), frame->getHeight());
            memcpy(rgb.data, frame->getData(), frame->getDataSize());
            rgb.flipY();
            rgb.upload();
            glDrawArrays(GL_TRIANGLES, 0, 6);
            rgb.deallocate();
        }
        if (frame->getType() == openni::SensorType::SENSOR_DEPTH)
        {
            renderShader.use();

            rgb.allocate(frame->getWidth(), frame->getHeight());
            memcpy(rgb.data, frame->getData(), frame->getDataSize());
            rgb.flipY();
            rgb.upload();
            glDrawArrays(GL_TRIANGLES, 0, 6);
            rgb.deallocate();
        }

        gl()->glDeleteBuffers(1, &triangle_vbo);
        gl()->glDeleteVertexArrays(1, &triangle_vao);
    }

    // put the stuff we've been drawing onto the display
    glfwSwapBuffers(window);
    // update other events like input handling 
    glfwPollEvents();

    return glfwWindowShouldClose(window);
}


void Viewer::addFrame(std::shared_ptr<Frame> frame, const int streamIdx)
{
    std::string id;
    openni::SensorType type = frame->getType();
    switch(type) {
        case openni::SensorType::SENSOR_IR:
            id = "IR";
            break;
        case openni::SensorType::SENSOR_DEPTH:
            id = "DEPTH";
            break;
        case openni::SensorType::SENSOR_COLOR:
            id = "RGB";
            break;
        default:
            logger->log(libfreenect2::Logger::Error, "SensorType not supported!");
            exit(0x0);
    }
    id += std::to_string(streamIdx);
    frames[id] = frame;
}


std::shared_ptr<Frame> Viewer::getFrame(openni::SensorType type, const int kinectIdx)
{
    std::string id;
    switch(type) {
        case openni::SensorType::SENSOR_IR:
            id = "IR";
            break;
        case openni::SensorType::SENSOR_DEPTH:
            id = "DEPTH";
            break;
        case openni::SensorType::SENSOR_COLOR:
            id = "RGB";
            break;
        default:
            logger->log(libfreenect2::Logger::Error, "SensorType not supported!");
            exit(0x0);
    } 
    id += std::to_string(kinectIdx);
    return frames[id];
}

bool Viewer::frameReady(openni::SensorType type, const int kinectIdx)
{
    std::string id;
    switch(type) {
        case openni::SensorType::SENSOR_IR:
            id = "IR";
            break;
        case openni::SensorType::SENSOR_DEPTH:
            id = "DEPTH";
            break;
        case openni::SensorType::SENSOR_COLOR:
            id = "RGB";
            break;
        default:
            logger->log(libfreenect2::Logger::Error, "SensorType not supported!");
            exit(0x0);
    } 
    id += std::to_string(kinectIdx);
    return frames.contains(id);
}

