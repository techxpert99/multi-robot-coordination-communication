// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from robot_communication_interfaces:srv/CommunicationService.idl
// generated code does not contain a copyright notice
#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#include <Python.h>
#include <stdbool.h>
#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-function"
#endif
#include "numpy/ndarrayobject.h"
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif
#include "rosidl_runtime_c/visibility_control.h"
#include "robot_communication_interfaces/srv/detail/communication_service__struct.h"
#include "robot_communication_interfaces/srv/detail/communication_service__functions.h"

bool robot_communication_interfaces__msg__communication_message__convert_from_py(PyObject * _pymsg, void * _ros_message);
PyObject * robot_communication_interfaces__msg__communication_message__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool robot_communication_interfaces__srv__communication_service__request__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[87];
    {
      char * class_name = NULL;
      char * module_name = NULL;
      {
        PyObject * class_attr = PyObject_GetAttrString(_pymsg, "__class__");
        if (class_attr) {
          PyObject * name_attr = PyObject_GetAttrString(class_attr, "__name__");
          if (name_attr) {
            class_name = (char *)PyUnicode_1BYTE_DATA(name_attr);
            Py_DECREF(name_attr);
          }
          PyObject * module_attr = PyObject_GetAttrString(class_attr, "__module__");
          if (module_attr) {
            module_name = (char *)PyUnicode_1BYTE_DATA(module_attr);
            Py_DECREF(module_attr);
          }
          Py_DECREF(class_attr);
        }
      }
      if (!class_name || !module_name) {
        return false;
      }
      snprintf(full_classname_dest, sizeof(full_classname_dest), "%s.%s", module_name, class_name);
    }
    assert(strncmp("robot_communication_interfaces.srv._communication_service.CommunicationService_Request", full_classname_dest, 86) == 0);
  }
  robot_communication_interfaces__srv__CommunicationService_Request * ros_message = _ros_message;
  {  // input_message
    PyObject * field = PyObject_GetAttrString(_pymsg, "input_message");
    if (!field) {
      return false;
    }
    if (!robot_communication_interfaces__msg__communication_message__convert_from_py(field, &ros_message->input_message)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * robot_communication_interfaces__srv__communication_service__request__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of CommunicationService_Request */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("robot_communication_interfaces.srv._communication_service");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "CommunicationService_Request");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  robot_communication_interfaces__srv__CommunicationService_Request * ros_message = (robot_communication_interfaces__srv__CommunicationService_Request *)raw_ros_message;
  {  // input_message
    PyObject * field = NULL;
    field = robot_communication_interfaces__msg__communication_message__convert_to_py(&ros_message->input_message);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "input_message", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}

#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
// already included above
// #include <Python.h>
// already included above
// #include <stdbool.h>
// already included above
// #include "numpy/ndarrayobject.h"
// already included above
// #include "rosidl_runtime_c/visibility_control.h"
// already included above
// #include "robot_communication_interfaces/srv/detail/communication_service__struct.h"
// already included above
// #include "robot_communication_interfaces/srv/detail/communication_service__functions.h"

bool robot_communication_interfaces__msg__communication_message__convert_from_py(PyObject * _pymsg, void * _ros_message);
PyObject * robot_communication_interfaces__msg__communication_message__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool robot_communication_interfaces__srv__communication_service__response__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[88];
    {
      char * class_name = NULL;
      char * module_name = NULL;
      {
        PyObject * class_attr = PyObject_GetAttrString(_pymsg, "__class__");
        if (class_attr) {
          PyObject * name_attr = PyObject_GetAttrString(class_attr, "__name__");
          if (name_attr) {
            class_name = (char *)PyUnicode_1BYTE_DATA(name_attr);
            Py_DECREF(name_attr);
          }
          PyObject * module_attr = PyObject_GetAttrString(class_attr, "__module__");
          if (module_attr) {
            module_name = (char *)PyUnicode_1BYTE_DATA(module_attr);
            Py_DECREF(module_attr);
          }
          Py_DECREF(class_attr);
        }
      }
      if (!class_name || !module_name) {
        return false;
      }
      snprintf(full_classname_dest, sizeof(full_classname_dest), "%s.%s", module_name, class_name);
    }
    assert(strncmp("robot_communication_interfaces.srv._communication_service.CommunicationService_Response", full_classname_dest, 87) == 0);
  }
  robot_communication_interfaces__srv__CommunicationService_Response * ros_message = _ros_message;
  {  // output_message
    PyObject * field = PyObject_GetAttrString(_pymsg, "output_message");
    if (!field) {
      return false;
    }
    if (!robot_communication_interfaces__msg__communication_message__convert_from_py(field, &ros_message->output_message)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * robot_communication_interfaces__srv__communication_service__response__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of CommunicationService_Response */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("robot_communication_interfaces.srv._communication_service");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "CommunicationService_Response");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  robot_communication_interfaces__srv__CommunicationService_Response * ros_message = (robot_communication_interfaces__srv__CommunicationService_Response *)raw_ros_message;
  {  // output_message
    PyObject * field = NULL;
    field = robot_communication_interfaces__msg__communication_message__convert_to_py(&ros_message->output_message);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "output_message", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
