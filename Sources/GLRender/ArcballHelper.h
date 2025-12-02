#ifndef _ARCBALL_HELPERH_H_
#define _ARCBALL_HELPERH_H_

//reference : https://en.wikibooks.org/wiki/OpenGL_Programming/Modern_OpenGL_Tutorial_Arcball
//https://asliceofrendering.com/camera/2019/11/30/ArcballCamera/

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>
#include <math.h>
#include <algorithm>
#include <vector>
#ifdef near
#undef near
#endif

#ifdef far
#undef far
#endif


namespace glm
{
    struct ray
    {
        glm::vec3 position;
        glm::vec3 direction; // normalized

        ray(const glm::vec3& pos, const glm::vec3& dir)
        {
            position = pos;
            direction = glm::normalize(dir);
        }

        ray()
        {
            position = glm::vec3(0, 0, 0);
            direction = glm::vec3(1, 0, 0);
        }

        glm::vec3 at(float t)
        {
            return position + direction * t;
        }

    };

    struct plane
    {
        glm::vec3 position;
        glm::vec3 normal;

        plane(const glm::vec3& position, const glm::vec3& normal)
        {
            this->position = position;
            this->normal = normal;
        }

        plane()
        {
            this->position = glm::vec3(0, 0, 0);
            this->normal = glm::vec3(0, 1, 0);
        }
    };

}

//class ShadowMap;
namespace Utility
{
    class ArcballCamera
    {
    public:
        ArcballCamera(float w, float h, const glm::vec3& eye = glm::vec3(0, 0, 3), const glm::vec3& target = glm::vec3(0, 0, 0), const glm::vec3& up = glm::vec3(0, 1, 0))
        {
            screen_width = w;
            screen_height = h;
            this->eye = eye;
            this->target = target;
            this->up = up;

            this->default_eye = eye;
            this->default_target = target;
            this->default_up = up;
        }

        glm::vec3 getArcballVector(float x, float y) {
            //glm::vec3 P = glm::vec3(1.0 * x / screen_width * 2 - 1.0,
            //    1.0 * y / screen_height * 2 - 1.0,
            //    0);

            glm::vec3 P = glm::vec3(x / screen_width * 2.0 - rot_center.x,
                y / screen_width * 2.0 - rot_center.y,
                0.0);

            P.y = -P.y;
            float OP_squared = P.x * P.x + P.y * P.y;
            if (OP_squared <= 1 * 1)
                P.z = sqrt(1 * 1 - OP_squared);  // Pythagoras
            else
                P = glm::normalize(P);  // nearest point
            return P;
        }

        glm::vec2 getScreenSize()
        {
            return glm::vec2(screen_width, screen_height);
        };

        //Frustum getFrustum()
        //{
        //    return Frustum();
        //};

        void update(float x, float y)
        {
            raw_mx = x;
            raw_my = y;

            auto& cube_rect = cube_gizmo_region;
            float t_x = x;
            float t_y = screen_height - 1 - y;
            bool no_pan = false;
            //if (t_x > cube_rect.x && t_x <cube_rect.x + cube_rect.z &&
            //    t_y>cube_rect.y && t_y < cube_rect.y + cube_rect.w
            //    )
            //{
            //    //remap x and y
            //    t_x = float(t_x - cube_rect.x) / cube_rect.z * screen_width;
            //    t_y = float(t_y - cube_rect.y) / cube_rect.w * screen_height;
            //    no_pan = true;

            //    x = t_x;
            //    y = screen_height - 1 - t_y;
            //}

            cur_mx = x;
            cur_my = y;


            if (left_down)
                rotate();

            if (right_down && !no_pan)
                pan();


            last_mx = cur_mx;
            last_my = cur_my;


        }

        void updateFrame(float delta_time = 1.0f / 60.0f)
        {
            total_time += delta_time;

            if (rotate_camera)
            {
                /*if (total_time >= rot_time_end)
                    rotate_camera = false;*/

                    //get interpolate quaterion
                float ratio = (total_time - rot_time_start) / (rot_time_end - rot_time_start);
                if (ratio > 1.0)
                    rotate_camera = false;
                auto rot_interpolate = glm::mix(rot_start, rot_end, ratio);
                ////update 
                //auto newdir = rot_interpolate * glm::vec3(1, 0, 0);
                //auto distance = glm::length(eye - target);
                //auto neweye = target + distance * newdir;
                //eye = neweye;

                //update up
                auto rot_up_interpolate = glm::mix(rot_up_start, rot_up_end, ratio);
                //up = rot_up_interpolate * glm::vec3(1, 0, 0);


                target = rot_interpolate * rotate_start_target;
                eye = rot_interpolate * rotate_start_eye;
                auto up_rotated = rot_up_interpolate * rotate_start_up;
                //update new eye
                up = glm::vec3(up_rotated);

                //printf("rotating [%f %f %f]\n", eye.x, eye.y, eye.z);
            }
        }
        //for left click
        int down_mx = 0, down_my = 0;
        void updateMouseState(int button, int state)
        {
            left_click = false;
            if (button == 0)
            {
                static bool pre_left_state = false;
                left_down = state;

                //record mouse down position
                if (left_down && left_down != pre_left_state)
                {
                    down_mx = raw_mx;
                    down_my = raw_my;

                    //printf("%d %d\n", down_mx, down_my);
                }
                else
                {
                    //check if this event is click
                    if (abs(down_mx - raw_mx) < 3 && abs(down_my - raw_my) < 3)
                        left_click = true;
                    else
                        left_click = false;
                }

                pre_left_state = left_down;
            }
            //if (button == 1 && shift == 1) middle_down = state;
            //else if (button == 1 && shift == 0) right_down = state;

            if (button == 2) middle_down = state;
            else if (button == 1)
            {
                right_down = state;
                //printf("set rightdown %s\n", state ? "true" : "false");
            }


            if (!state)
            {
                middle_down = false;
                right_down = false;
            }


            if (left_down)
            {
                //here rember rotate state
                //rot_center = glm::project(glm::vec3(0, 0, 0), getView(), getProj(),glm::vec4(0,0,screen_width,screen_height));
                //rot_center.x = rot_center.x * 2.0f / screen_width;
                //rot_center.y = rot_center.y * 2.0f / screen_height;


            }
        }


        //!short cut function
        void inputMouseScroll(int x)
        {
            zoomAt(x > 0);
        }

        void inputMouse(int x, int y, int mouse_button, int mouse_press_or_release)
        {

            update(x, y);
            if (mouse_button == 0 && mouse_press_or_release)
            {
                updateMouseState(0, 1);
            }

            if (mouse_button == 0 && !mouse_press_or_release)
            {
                updateMouseState(0, 0);
            }

            //right mouse
            if (mouse_button == 1 && mouse_press_or_release)
            {
                updateMouseState(1, 1);
            }
            if (mouse_button == 1 && !mouse_press_or_release)
            {
                updateMouseState(1, 0);
            }

        }


        void rotate()
        {
            if (cur_mx != last_mx || cur_my != last_my) {
                glm::vec3 va = getArcballVector(last_mx, last_my);
                glm::vec3 vb = getArcballVector(cur_mx, cur_my);
                float angle = acos(std::min<float>(1.0f, glm::dot(va, vb)));

                glm::vec3 axis_in_camera_coord = glm::normalize(glm::cross(vb, va));
#if 0
                //rotate around center
                glm::vec3 fake_target = target;
                //printf("axis{ %f %f %f},angle{%f}\n",
                //    axis_in_camera_coord.x, axis_in_camera_coord.y, axis_in_camera_coord.z,
                //    angle);

                auto transform = getView();
                auto transform_inv = glm::inverse(transform);

                //mesh point -> camera view(transform) ->rotate in camera axis ->world view(transform_inv)
                glm::mat3 rot_mat = transform_inv * glm::rotate(glm::mat4(1.0f), angle, axis_in_camera_coord) * transform;

                auto offset = eye - fake_target;
                auto offset_rotated = rot_mat * offset;
                auto up_rotated = rot_mat * up;
                //update new eye
                eye = glm::vec3(offset_rotated) + fake_target;
                up = glm::vec3(up_rotated);
#else
                //rotate around object center
                glm::vec3 fake_target = target;

                auto transform = getView();
                auto transform_inv = glm::inverse(transform);

                //解决问题：转换后的相机位置和目标将会是在哪里了？
                //从右到左：将相机eye和target转换到 相机空间  ---> 进行arcball旋转 ---->转换回world坐标
                //参见transform_cameara.png
                //这里有一点需要说明，这里假定了目标中心为（0，0，0）
                glm::mat3 rot_mat = transform_inv * glm::rotate(glm::mat4(1.0f), angle, axis_in_camera_coord) * transform;

                target = rot_mat * fake_target;
                eye = rot_mat * eye;
                auto up_rotated = rot_mat * up;
                //update new eye
                up = glm::vec3(up_rotated);
#endif

                //printf("mouse pos%f %f %f\n", eye.x, eye.y, eye.z);

                last_mx = cur_mx;
                last_my = cur_my;
            }
        }


        /// <summary>
        /// 
        /// </summary>
        /// <param name="newdir">direction from center(0,0,0) to this view ( or called it Normal)</param>
        /// <param name="time">in second</param>
        void rotateTo(const glm::vec3& newdir, float time = 0.5f)
        {
            //debug test
            glm::vec3 fake_target = target;

            auto transform = getView();
            auto transform_inv = glm::inverse(transform);

            auto cur_dir = glm::normalize(eye - target);
            auto axis_dir = glm::normalize(glm::cross(cur_dir, newdir));
            auto dot_val = glm::dot(cur_dir, glm::normalize(newdir));
            if (dot_val > 0.99) return;//these are two parralls dir
            auto angle = acos(dot_val);
            //解决问题：转换后的相机位置和目标将会是在哪里了？
            //从右到左：将相机eye和target转换到 相机空间  ---> 进行arcball旋转 ---->转换回world坐标
            //参见transform_cameara.png
            //这里有一点需要说明，这里假定了目标中心为（0，0，0）
            glm::mat3 rot_mat = glm::rotate(glm::mat4(1.0f), angle, axis_dir);
            //transform_inv* glm::rotate(glm::mat4(1.0f), angle, axis_dir)* transform;

        //target = rot_mat * fake_target;
        //eye = rot_mat * eye;
        //auto up_rotated = rot_mat * up;
        ////update new eye
        //up = glm::vec3(up_rotated);

        //return;

            rot_time_start = total_time;
            rot_time_end = rot_time_start + time;

            rotate_start_eye = eye;
            rotate_start_target = target;
            rotate_start_up = up;

            rot_start = glm::quat(1.0f, 0, 0, 0);
            rot_end = glm::quat(rot_mat);

            rot_up_start = glm::quat(1.0f, 0, 0, 0);
            glm::vec3 target_final_up = default_up;// glm::vec3(0, 1, 0);
            if (abs(glm::dot(newdir, default_up)) > 0.9999f)
            {
                //bottom or top
                target_final_up = glm::vec3(default_up.x, default_up.z, default_up.y);//swizzle y z
            }
            //default_final_up
            //glm::vec3 no_fix_final_up = rot_end * up;


            //auto getQuatFromTwoVectors = [](const glm::vec3& v1, const glm::vec3& v2) {

            //    if (glm::dot(glm::normalize(v1), glm::normalize(v2)) > 0.9999f)
            //        return glm::quat(1, 0, 0, 0);

            //    auto a = glm::normalize(glm::cross(v1, v2));

            //    float w = glm::length(v1) * glm::length(v2) + glm::dot(v1, v2);
            //    glm::quat q(w, a.x, a.y, a.z);
            //    q = glm::normalize(q);
            //    return q;
            //};

            auto orthogonal = [](glm::vec3 v)
            {
                float x = abs(v.x);
                float y = abs(v.y);
                float z = abs(v.z);

                glm::vec3 other = x < y ? (x < z ? glm::vec3(1, 0, 0) : glm::vec3(0, 0, 1)) : (y < z ? glm::vec3(0, 1, 0) : glm::vec3(0, 0, 1));
                return cross(v, other);
            };

            auto get_rotation_between = [&](const glm::vec3& uu, const glm::vec3& vv)
            {
                // It is important that the inputs are of equal length when
                // calculating the half-way vector.
                auto u = glm::normalize(uu);
                auto v = glm::normalize(vv);

                // Unfortunately, we have to check for when u == -v, as u + v
                // in this case will be (0, 0, 0), which cannot be normalized.
                if (u == -v)
                {
                    // 180 degree rotation around any orthogonal vector
                    return glm::quat(0, glm::normalize(orthogonal(u)));
                }

                glm::vec3 half = glm::normalize(u + v);
                return glm::quat(glm::dot(u, half), glm::cross(u, half));
            };

            rot_up_end = get_rotation_between(up, target_final_up);//glm::quat(rot_mat);

            //target = rot_mat * fake_target;
            //eye = rot_mat * eye;
            //auto up_rotated = rot_up_end * up;
            ////update new eye
            //up =  glm::vec3(up_rotated);//target_final_up;

            //return;


            //calculate target up
            //auto final_target = rot_mat * fake_target;
            //auto final_eye = rot_mat * eye;
            //auto final_up_rotated = rot_mat * up;

            //auto final_dir = final_target - final_eye;
            //auto final_right = glm::cross(final_dir, glm::vec3(0,1,0));
            //auto final_up = glm::normalize(glm::cross(final_right, final_dir));
            //auto up_rot = glm::quat(1.0f, 0, 0, 0);
            //{
            //    auto axis_dir = glm::normalize(glm::cross(final_up, final_up_rotated));
            //    auto dot_val = glm::dot(final_up, glm::normalize(final_up_rotated));
            //    auto angle = acos(dot_val);
            //    auto up_rot_mat = glm::rotate(glm::mat4(1.0f), angle, axis_dir);

            //    if (dot_val < 0.99)
            //    {
            //        up_rot = glm::quat(up_rot_mat);
            //    }
            //}

            //rot_up_start = glm::quat(1.0f,0, 0, 0);
            //rot_up_end = up_rot * rot_up_start;// glm::quat(rot_mat);

            rotate_camera = true;
            //return;
            //rot_time_start = total_time;
            //rot_time_end = rot_time_start + time;

            //auto direction = glm::normalize(eye - target);

            ////https://gamedev.stackexchange.com/questions/149006/direction-vector-to-quaternion

            //auto getQuatFromDirection = [](const glm::vec3& v) {
            //    //auto angle = atan2(v.x, v.z);
            //    //glm::quat rot;
            //    //rot.x = 0;
            //    //rot.y = 1 * sin(angle / 2);
            //    //rot.z = 0;
            //    //rot.w = cos(angle / 2);

            //    auto axis_dir = glm::normalize(glm::cross(glm::vec3(1, 0, 0), v));
            //    auto angle = acos(glm::dot(glm::vec3(1, 0, 0), glm::normalize(v)));
            //    auto rot = glm::angleAxis(angle, axis_dir);
            //    return rot;
            //};

            //rot_start = getQuatFromDirection(direction);
            //rot_end =  getQuatFromDirection(glm::normalize(newdir));


            //rot_up_start = getQuatFromDirection(up);


            //auto lookdir = glm::normalize(-newdir);
            //auto sidedir = glm::cross(lookdir, default_up);
            //auto endup = glm::normalize(glm::cross(sidedir, lookdir));

            //rot_up_end = getQuatFromDirection(endup);

            //rotate_camera = true;
        }

        void pan()
        {
            float x = cur_mx - last_mx;
            float y = cur_my - last_my;
            auto dir = eye - target;
            auto right_vector = glm::normalize(glm::cross(dir, up));

            float scale = glm::distance(eye, target) * 0.1f;

            auto offset = 0.01f * scale * (x * right_vector + y * up);
            eye += offset;
            target += offset;


            last_mx = cur_mx;
            last_my = cur_my;
        }

        void zoomAt(bool zoomin)
        {

            float scale = glm::distance(eye, target) * 0.1f;
            //auto factor = 1.0f;
            auto middle_vec = glm::vec3(cur_mx, screen_height - 1 - cur_my, 0.5);

            //printf("zoom:%s mx:%f my:%f\n", zoomin?"in":"out", mx, my);

            auto view = getView();
            auto proj = getProj();
            //double viewports[4];
            //glGetDoublev(GL_VIEWPORT, viewports);
            glm::vec4 viewport(0, 0, screen_width, screen_height);// viewports[2], viewports[3]);
            auto middle_3d = glm::unProject(middle_vec, view, proj, viewport);

            //p = proj * view * v;
            //auto v = glm::vec3(glm::inverse(proj * view) * glm::vec4(middle_vec,1.0));
            auto offset = glm::normalize(middle_3d - eye) * scale;

            if (zoomin)
            {
                eye += offset;
                target += offset;
            }
            else
            {
                eye -= offset;
                target -= offset;
            }
        }
        float last_mx = 0, last_my = 0, cur_mx = 0, cur_my = 0;
        float raw_mx = 0, raw_my = 0;
        float distance_to_gizmo_cube = 2.5f;
        int arcball_on = false;
        bool left_down = false;
        bool left_click = false;
        bool right_down = false;
        bool middle_down = false;

        float screen_width, screen_height;
        float near = 0.1f;
        float far = 2500.0f;

        float total_time = 0.0f;

        bool rotate_camera = false;
        float rot_time_start = 0.0f;
        float rot_time_end = 0.0f;
        glm::quat rot_start, rot_end;
        glm::quat rot_up_start, rot_up_end;

        glm::vec3 rot_center = glm::vec3(1, 1, 0);

        glm::vec3 eye, target, up, eye_start, target_start, up_start;
        //for rotating
        glm::vec3 rotate_start_eye, rotate_start_target, rotate_start_up;
        glm::vec3 default_eye, default_target, default_up;

        glm::vec4 cube_gizmo_region = glm::vec4(0, 0, 128, 128);

        glm::mat4 getView()
        {
            return glm::lookAt(eye, target, up);
        }

        void lookAt(const glm::vec3& pos, const glm::vec3& target, const glm::vec3& up)
        {
            this->eye = pos;
            this->target = target;
            this->up = up;
        }

        glm::vec3 getPosition()
        {
            return eye;
        }

        glm::mat4 getViewCube()
        {
            auto dir = glm::normalize(eye - target) * distance_to_gizmo_cube;

            return glm::lookAt(dir, glm::vec3(0, 0, 0), up);
        }

        glm::mat4 getProj()
        {
            return glm::perspective(glm::radians(45.0f), screen_width / screen_height, near, far);
        }
        glm::mat4 getProjCube()
        {
            return glm::perspective(glm::radians(45.0f), 1.0f, near, far);
        }

        glm::vec4 getViewport()
        {
            return glm::vec4(0, 0, screen_width, screen_height);
        }

        void setViewSize(int x, int y, int w, int h, bool is_whole_screen = true)
        {
            screen_width = (float)w;
            screen_height = (float)h;

            if (is_whole_screen)
            {
                const int size = 128;
                cube_gizmo_region = glm::vec4(screen_width - 1 - size, screen_height - 1 - size, size, size);
            }
        };

        glm::ray getRay(float mouseX, float mouseY)
        {
            //from web
            glm::vec2 ray_nds = glm::vec2(mouseX, mouseY);
            glm::vec4 ray_clip = glm::vec4(ray_nds.x, ray_nds.y, -1.0f, 1.0f);
            glm::mat4 invProjMat = glm::inverse(getProj());
            glm::vec4 eyeCoords = invProjMat * ray_clip;
            eyeCoords = glm::vec4(eyeCoords.x, eyeCoords.y, -1.0f, 0.0f);
            glm::mat4 invViewMat = glm::inverse(getView());
            glm::vec4 rayWorld = invViewMat * eyeCoords;
            glm::vec3 rayDirection = glm::normalize(glm::vec3(rayWorld));

            glm::ray ray;
            ray.position = eye;
            ray.direction = rayDirection;
            return ray;

        };

        glm::ray getRayCube(float mouseX, float mouseY)
        {
            //from web
            glm::vec2 ray_nds = glm::vec2(mouseX, mouseY);
            glm::vec4 ray_clip = glm::vec4(ray_nds.x, ray_nds.y, -1.0f, 1.0f);
            glm::mat4 invProjMat = glm::inverse(getProjCube());
            glm::vec4 eyeCoords = invProjMat * ray_clip;
            eyeCoords = glm::vec4(eyeCoords.x, eyeCoords.y, -1.0f, 0.0f);
            glm::mat4 invViewMat = glm::inverse(getViewCube());
            glm::vec4 rayWorld = invViewMat * eyeCoords;
            glm::vec3 rayDirection = glm::normalize(glm::vec3(rayWorld));

            glm::ray ray;
            ray.position = glm::normalize(eye - target) * distance_to_gizmo_cube;
            ray.direction = rayDirection;
            return ray;

        };
    };

}
#endif