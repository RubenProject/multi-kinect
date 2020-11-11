#include "viewer.h"
#include <cstdlib>
#include <cmath>


Viewer::Viewer() : shader_folder("src/shader/"), 
                   win_width(600),
                   win_height(400)
{
}

static void glfwErrorCallback(int error, const char* description)
{
  std::cerr << "GLFW error " << error << " " << description << std::endl;
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

    std::string vertexshadersrc = ""
        "#version 330\n"
                                                
        "in vec2 Position;"
        "in vec2 TexCoord;"
                    
        "out VertexData{"
        "vec2 TexCoord;" 
        "} VertexOut;"  
                    
        "void main(void)"
        "{"
        "    gl_Position = vec4(Position, 0.0, 1.0);"
        "    VertexOut.TexCoord = TexCoord;"
        "}";
    std::string grayfragmentshader = ""
        "#version 330\n"
        
        "uniform sampler2DRect Data;"
        
        "vec4 tempColor;"
        "in VertexData{"
        "    vec2 TexCoord;"
        "} FragmentIn;"
        
        "layout(location = 0) out vec4 Color;"
        
        "void main(void)"
        "{"
            "ivec2 uv = ivec2(FragmentIn.TexCoord.x, FragmentIn.TexCoord.y);"
            "tempColor = texelFetch(Data, uv);"
            "Color = vec4(tempColor.x/4500, tempColor.x/4500, tempColor.x/4500, 1);"
        "}";
    std::string fragmentshader = ""
        "#version 330\n"
        
        "uniform sampler2DRect Data;"
        
        "in VertexData{"
        "    vec2 TexCoord;"
        "} FragmentIn;"
       
        "layout(location = 0) out vec4 Color;"
        
        "void main(void)"
        "{"
        "    ivec2 uv = ivec2(FragmentIn.TexCoord.x, FragmentIn.TexCoord.y);"

        "    Color = texelFetch(Data, uv);"
        "}";

    renderShader.setVertexShader(vertexshadersrc);
    renderShader.setFragmentShader(fragmentshader);
    renderShader.build();

    renderGrayShader.setVertexShader(vertexshadersrc);
    renderGrayShader.setFragmentShader(grayfragmentshader);
    renderGrayShader.build();


    glfwSetWindowUserPointer(window, this);
    glfwSetKeyCallback(window, Viewer::key_callbackstatic);
    glfwSetWindowSizeCallback(window, Viewer::winsize_callbackstatic);

    shouldStop = false;
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

void Viewer::key_callbackstatic(GLFWwindow* window, int key, int scancode, int action, int mods)
{
    Viewer* viewer = reinterpret_cast<Viewer*>(glfwGetWindowUserPointer(window));
    viewer->key_callback(window, key, scancode, action, mods);
}

void Viewer::key_callback(GLFWwindow* window, int key, int scancode, int action, int mods)
{
    if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS)
        shouldStop = true;
}

void Viewer::onOpenGLBindingsChanged(OpenGLBindings *b)
{
    renderShader.gl(b);
    renderGrayShader.gl(b);
    rgb.gl(b);
    ir.gl(b);
}

void Viewer::convert_frame(std::shared_ptr<openni::VideoFrameRef> in_frame, libfreenect2::Frame** out_frame)
{
    const int width = in_frame->getWidth();
    const int height = in_frame->getHeight();
    float* data = new float[width * height];
    uint16_t* grey_value = (uint16_t*)in_frame->getData();
    for (int i = 0; i < width * height; i++){
        data[i] = (float)grey_value[i];
    }
    *out_frame = new libfreenect2::Frame(width, height, 4, (unsigned char*)data);
}

bool Viewer::render()
{
    // wipe the drawing surface clear
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    GLint x = 0, y = 0;
    int fb_width, fb_width_half, fb_height, fb_height_half;

    for (auto iter = frames.begin(); iter != frames.end(); ++iter)
    {
        Frame *frame = iter->second;

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


        if (iter->first == "RGB0" || iter->first == "RGB1")
        {
            renderShader.use();

            rgb.allocate(frame->getWidth(), frame->getHeight());
            memcpy(rgb.data, frame->getData(), frame->getDataSize());
            rgb.flipY();
            rgb.upload();
            glDrawArrays(GL_TRIANGLES, 0, 6);
            rgb.deallocate();
        }
        if (iter->first == "IR0" || iter->first == "IR1")
        {
            renderGrayShader.use();

            ir.allocate(frame->getWidth(), frame->getHeight());
            memcpy(ir.data, frame->getData(), frame->getDataSize());
            ir.flipY();
            ir.upload();
            glDrawArrays(GL_TRIANGLES, 0, 6);
            ir.deallocate();
        }
        if (iter->first == "BODY0" || iter->first == "BODY1")
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

    return shouldStop || glfwWindowShouldClose(window);
}

void Viewer::addFrame(std::string id, Frame *frame)
{
    delete frames[id];
    frames[id] = frame;
}
