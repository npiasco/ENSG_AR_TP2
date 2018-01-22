// Std. Includes
#include <string>
#include <cmath>
#include <ctime>
#include <stdlib.h>     /* srand, rand */

// GLEW
#include <GL/glew.h>

// GLFW
#include <GLFW/glfw3.h>

// GL includes
#include "Shader.h"
#include "Model.h"

// GLM Mathemtics
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include "opencv2/opencv.hpp"
#include<System.h>

/**
 * Convert cvmat cv::Mat type in glmmat glm::mat4 type
 * */
void fromCV2GLM(const cv::Mat& cvmat, glm::mat4& glmmat) {
    if (cvmat.cols != 4 || cvmat.rows != 4 || cvmat.type() != CV_32FC1) {
        cout << "Matrix conversion error!" << endl;
        return;
    }
    memcpy(glm::value_ptr(glmmat), cvmat.data, 16 * sizeof(float));
    glmmat = glm::transpose(glmmat);
}

/**
 * Convert cvmat cv::Mat type in glmmat glm::vec3 type
 * */
void fromCV2GLM(const cv::Mat& cvmat, glm::vec3& glmmat) {
    if (cvmat.cols != 1 || cvmat.rows != 3 || cvmat.type() != CV_32FC1) {
        cout << "Matrix conversion error!" << endl;
        return;
    }
    memcpy(glm::value_ptr(glmmat), cvmat.data, 3 * sizeof(float));
}

void key_callback(GLFWwindow* window, int key, int scancode, int action, int mode);

int main(int argc, char **argv)
{
    // Initialisation du systeme de tracking, mettre false pour desactiver le gui
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,true);
    cv::VideoCapture cap(0);
    cap.set(CV_CAP_PROP_FRAME_HEIGHT,480);
    cap.set(CV_CAP_PROP_FRAME_WIDTH,640);

    /************ Initialisation Fenetre, Shaders, texture ***********/
    GLuint screenWidth = 800, screenHeight = 600;

    glfwInit();
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_RESIZABLE, GL_FALSE);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
    
    GLFWwindow* window = glfwCreateWindow(screenWidth, screenHeight, "OpenGL", nullptr, nullptr); // Windowed
    glfwMakeContextCurrent(window);
    glfwSetKeyCallback(window, key_callback);

    glewExperimental = GL_TRUE;
    glewInit();
    
    // Define the viewport dimensions
    int width, height;
    // On recupere les dimensions de la fenetre creee plus haut
    glfwGetFramebufferSize(window, &width, &height);
    glViewport(0, 0, width, height);
    
    glEnable(GL_DEPTH_TEST);
    
    Shader shader("opengl_code/shaders/default.vertexshader", "opengl_code/shaders/default.fragmentshader");
    Shader lightshader("opengl_code/shaders/light.vertexshader", "opengl_code/shaders/light.fragmentshader");
            
    GLuint texture; // Declaration de l'identifiant

	glGenTextures(1, &texture); // Generation de la texture
	// On bind la texture cree dans le contexte global d'OpenGL
	glBindTexture(GL_TEXTURE_2D, texture); 
	// Modification des parametres de la texture
	// Methode de wrapping
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT); 
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT); 
	// Methode de filtrage
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR); 
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

    // On unbind la texture
	glBindTexture(GL_TEXTURE_2D, 0);

    /* A MODIFIER */
    GLfloat fov=43.13f;
	
    glm::mat4 projection = glm::perspective((GLfloat) (fov*M_PI/180.0), 4.0f/3.0f, 0.1f, 300.0f);

    shader.Use();
    glUniformMatrix4fv(glGetUniformLocation(shader.Program, "projection"), 1, GL_FALSE, glm::value_ptr(projection));
    lightshader.Use();
    glUniformMatrix4fv(glGetUniformLocation(lightshader.Program, "projection"), 1, GL_FALSE, glm::value_ptr(projection));

    GLfloat focal = projection[0][0]; 

	GLfloat vertices[] = {
       /*      Positions    |      Normales     |     UV     */
       1.0f,  0.75f, 0.0f,   0.0f, 0.0f, 0.0f,   1.0f, 1.0f, // Top Right
       1.0f, -0.75f, 0.0f,   0.0f, 0.0f, 0.0f,   1.0f, 0.0f, // Bottom Right
      -1.0f, -0.75f, 0.0f,   0.0f, 0.0f, 0.0f,   0.0f, 0.0f, // Bottom Left
      -1.0f,  0.75f, 0.0f,   0.0f, 0.0f, 0.0f,   0.0f, 1.0f  // Top Left
    };
        
    GLshort indices[]{
    	0, 1, 3,
    	1, 2, 3
    };    
    
    GLuint VBO, VAO, EBO;
    
    glGenVertexArrays(1, &VAO);
    
    glGenBuffers(1, &VBO);
    glGenBuffers(1, &EBO);
    
    glBindVertexArray(VAO);
    
    // On met notre EBO dans le contexte global
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
    // On assigne au EBO le tableau d'indices
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(indices), indices, GL_STATIC_DRAW); 

    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);

    // Attribut des positions
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(GLfloat), (GLvoid*)0);
    glEnableVertexAttribArray(0);
    // Attribut des normales
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(GLfloat), (GLvoid*)(3*sizeof(GLfloat)));
    glEnableVertexAttribArray(1);
    // Attribut des coord
    glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, 8 * sizeof(GLfloat), (GLvoid*)(6*sizeof(GLfloat)));
    glEnableVertexAttribArray(2);
    
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);
    

    // Chargement des modèles
    std::vector<Model> Models;
    Models.push_back(Model("opengl_code/model/House/house.obj")); // Num 0 
    Models.push_back(Model("opengl_code/model/Lamp/Lamp.obj")); // Num 1 
    Models.push_back(Model("opengl_code/model/Trees/Tree1/Tree1.3ds")); // Num 2 
    Models.push_back(Model("opengl_code/model/sphere/sphere.stl")); // Num 3 
    Models.push_back(Model("opengl_code/model/suzanne/suzanne.obj")); // Num 4 

    // Game loop
    cv::Mat image;
    while(!glfwWindowShouldClose(window))
    {
        glClear(GL_DEPTH_BUFFER_BIT); // Remise à zero du Z-buffer

        cap >> image;
        cv::flip(image, image, 0);

        glfwPollEvents();
        glClearColor(0.05f, 0.05f, 0.05f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);

        /* A MODIFIER */
        glm::mat4 view(1.0f), model(1.0f);


        lightshader.Use();
        glUniformMatrix4fv(glGetUniformLocation(lightshader.Program, "view"), 1, GL_FALSE, glm::value_ptr(view));
        shader.Use();
        glUniformMatrix4fv(glGetUniformLocation(shader.Program, "view"), 1, GL_FALSE, glm::value_ptr(view));
        glUniformMatrix4fv(glGetUniformLocation(shader.Program, "model"), 1, GL_FALSE, glm::value_ptr(model));
        
		//  Activiation  de la  texture 0
		glActiveTexture(GL_TEXTURE0 );//  Binding  de  notre  texture
		glBindTexture(GL_TEXTURE_2D , texture );//  Association du numero de la texture  pour le  shader
        
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, image.cols, image.rows,
                     0, GL_BGR, GL_UNSIGNED_BYTE, image.ptr());

		glUniform1i(glGetUniformLocation(shader.Program , "modelTexture"), 0);
        
        glBindVertexArray(VAO);
        // On dessine l'objet courant 
        glDrawElements(GL_TRIANGLES, 3*2, GL_UNSIGNED_SHORT, 0);
        glBindVertexArray(0);

        /**************             Affichage de l'envrionnement augmenté               *****************/

        /* A MODIFIER */



        

        glfwSwapBuffers(window);
    }
    
    glDeleteVertexArrays(1, &VAO);
    glDeleteBuffers(1, &VBO);
    glDeleteBuffers(1, &EBO);
    
    // Stop all threads
    SLAM.Shutdown();
    
    glfwTerminate();
    return 0;
}


void key_callback(GLFWwindow* window, int key, int scancode, int action, int mode)
{
    if(key == GLFW_KEY_ESCAPE && action == GLFW_PRESS)
        glfwSetWindowShouldClose(window, GL_TRUE);
}
