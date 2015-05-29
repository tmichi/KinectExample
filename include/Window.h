#ifndef WINDOW_H
#define WINDOW_H 1
#include <GLFW/glfw3.h>

class Window
{
private:
        GLFWwindow* window_;
        double oldx_;
        double oldy_;
private:
        void operator = ( const Window& that ) ;
        Window( const Window& that);

public:
        explicit Window ( const int width, const int height, const char* windowName = "Hello World" ) 
                : oldx_( 0 ), oldy_( 0 )
        {
                this->window_ =  glfwCreateWindow( width, height, windowName, NULL, NULL );
                glfwMakeContextCurrent( this->window_ );
                glfwSetFramebufferSizeCallback( this->window_, Window::resize );
        }

        virtual ~Window( void ) {
                glfwTerminate();
                return;

        }

        bool operator ! ( void ) {
                return ! ( this->window_ );
        }

        GLFWwindow* getWindow() {
                return this->window_;
        }

        void getMousePosition( double& ox, double& oy, double& nx, double& ny ) {
                ox = this->oldx_;
                oy = this->oldy_;

                double mx, my;
                int w, h;
                glfwGetCursorPos( this->window_, &mx, &my );
                glfwGetWindowSize( this->window_, &w, &h );
                const int r = w < h ? w : h;

                nx = ( 2.0 * mx - w ) / r;
                ny = ( h - 2.0 * my ) / r;

                this->oldx_ = nx;
                this->oldy_ = ny;
                return;

        }

        static void resize ( GLFWwindow* window, const int width, const int height ){
                glfwMakeContextCurrent( window );
                glViewport( 0, 0, width, height );
                return;
        }

        double getAspectRatio( void ) const {
                int w, h;
                glfwGetWindowSize( this->window_, &w, &h );
                return  w * 1.0 / h ;
        }
};
#endif // WINDOW_H


