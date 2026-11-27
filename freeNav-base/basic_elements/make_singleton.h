//
// Created by yaozhuo on 2022/1/11.
//

#ifndef FREENAV_BASE_MAKE_SINGLETON_H
#define FREENAV_BASE_MAKE_SINGLETON_H
namespace freeNav {

#define MAKE_SINALETON(class_name)                    \
    class_name* class_name::_instance = NULL;         \
    class_name* class_name::getInstance()             \
    {                                                 \
        if(class_name::_instance == NULL)             \
        {                                             \
            class_name::_instance = new class_name(); \
        }                                             \
        return class_name::_instance;                 \
    }
}

#endif //FREENAV_MAKE_SINGLETON_H
