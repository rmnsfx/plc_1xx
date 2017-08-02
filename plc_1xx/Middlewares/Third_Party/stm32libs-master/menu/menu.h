/**
 * @file menu.h Библиотека работы с меню.
 */

#ifndef MENU_H
#define	MENU_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "defs/defs.h"
#include "errors/errors.h"
#include "fixed/fixed32.h"



//! Список типов значений элементов меню.
typedef enum _MenuValueType {
    MENU_VALUE_TYPE_NONE = 0, //!< Нет значения.
    MENU_VALUE_TYPE_STRING = 1, //!< Строка.
    MENU_VALUE_TYPE_INT = 2, //!< Целочисленное.
    MENU_VALUE_TYPE_BOOL = 3, //!< Логическое.
    MENU_VALUE_TYPE_FIXED = 4, //!< Число с фиксированной запятой.
    MENU_VALUE_TYPE_ENUM = 5, //!< Списочное.
    MENU_VALUE_TYPE_CUSTOM = 6 //!< Пользовательское.
} menu_value_type_t;


// Предварительная декларация типа значения.
struct _MenuValue;

//! Текстовое значение.
typedef const char* menu_value_string_t;
//! Целочисленное значение.
typedef int32_t menu_value_int_t;
//! Логическое значение.
typedef bool menu_value_bool_t;
//! Число с фиксированной запятой.
typedef fixed32_t menu_value_fixed_t;
//! Списочное значение.
typedef struct _MenuValueEnum {
    struct _MenuValue* values; //!< Значения.
    size_t count; //!< Число значений.
    size_t current; //!< Текущее значение.
} menu_value_enum_t;
//! Пользовательское.
typedef void* menu_value_custom_t;

//! Общий тип значения элемента меню.
typedef struct _MenuValue {
    menu_value_type_t type; //!< Тип значения.
    union {
        menu_value_string_t value_string; //!< Текстовое значение.
        menu_value_int_t value_int; //!< Целочисленное значение.
        menu_value_bool_t value_bool; //!< Логическое значение.
        menu_value_fixed_t value_fixed; //!< Число с фиксированной запятой.
        menu_value_enum_t value_enum; //!< Списочное значение.
        menu_value_custom_t value_custom; //!< Пользовательское значение.
    };
} menu_value_t;


//! Тип идентификатора меню.
typedef uint16_t menu_id_t;
//! Тип иденетификатора иконки.
typedef uint16_t menu_icon_id_t;
//! Тип флагов меню.
typedef uint32_t menu_flags_t;
//! Тип пользовательских данных меню.
typedef void* menu_user_data_t;

//! Предварительная декларация типа элемента меню.
struct _MenuItem;

//! Тип меню.
typedef struct _Menu {
    struct _MenuItem* root; //!< Корневой элемент.
    struct _MenuItem* current; //!< Текущий элемент.
} menu_t;

//! Тип элемента меню.
typedef struct _MenuItem {
    menu_id_t id; //!< Идентификатор.
    
    struct _MenuItem* parent; //!< Родительский элемент.
    struct _MenuItem* child; //!< Дочерний элемент.
    struct _MenuItem* prev; //!< Предыдущий элемент.
    struct _MenuItem* next; //!< Следующий элемент.
    
    const char* text; //!< Текст.
    const char* help; //!< Справка.
    
    menu_icon_id_t icon_id; //!< Идентификатор иконки.
    
    menu_value_t* value; //!< Значение.
    
    menu_flags_t flags; //!< Флаги.
    
    menu_user_data_t user_data; //!< Пользовательские данные.
} menu_item_t;


//! Тип глубины элемента меню.
typedef uint8_t menu_depth_t;

//! Тип структуры дескриптора элемента меню.
typedef struct _MenuDescr {
    menu_depth_t depth; //!< Глубина элемента меню.
    menu_id_t id; //!< Идентификатор элемента меню.
    const char* text; //!< Текст элемента меню.
    const char* help; //!< Справка элемента меню.
    menu_icon_id_t icon_id; //!< Идентификатор иконки элемента меню.
    menu_flags_t flags; //!< Флаги элемента меню.
    menu_user_data_t user_data; //!< Пользовательские данные элемента меню.
    menu_value_t* value; //!< Значение элемента меню.
} menu_descr_t;


#ifdef MENU_MACRO

/**
 * Атрибут для переменной меню.
 */
#ifndef MENU_ATTRIBS
#define MENU_ATTRIBS
#endif

/**
 * Атрибут для переменной значения элемента меню.
 */
#ifndef MENU_VALUE_ATTRIBS
#define MENU_VALUE_ATTRIBS
#endif

/**
 * Атрибут для массива значений элемента меню.
 */
#ifndef MENU_VALUE_ARRAY_ATTRIBS
#define MENU_VALUE_ARRAY_ATTRIBS
#endif

/**
 * Атрибут для переменной элемента меню.
 */
#ifndef MENU_ITEM_ATTRIBS
#define MENU_ITEM_ATTRIBS
#endif

/**
 * Постфикс для генерируемых имён переменных значений элементов меню.
 */
#ifndef MENU_VALUE_NAME_POSTFIX
#define MENU_VALUE_NAME_POSTFIX _value
#endif

//! Декларирует меню.
#define DECLARE_MENU(name) MENU_ATTRIBS menu_t name
//! Декларирует несколько меню.
#define DECLARE_MENUS(...) MENU_ATTRIBS menu_t __VA_ARGS__
//! Инициализация структуры меню.
#define MAKE_MENU(arg_root) { .root = (menu_item_t*)arg_root, .current = (menu_item_t*)arg_root }
//! Обявляет и инициализирует переменную меню.
#define MENU(name, arg_root) DECLARE_MENU(name) = MAKE_MENU(arg_root)


//! Декларирует значение элемента меню.
#define DECLARE_MENU_VALUE(name) MENU_VALUE_ATTRIBS menu_value_t name
//! Декларирует несколько значений элемента меню.
#define DECLARE_MENU_VALUES(...) MENU_VALUE_ATTRIBS menu_value_t __VA_ARGS__
//! Декларирует массив значений элемента меню.
#define DECLARE_MENU_VALUES_ARRAY(name) MENU_VALUE_ARRAY_ATTRIBS menu_value_t name[]
//! Инициализирует массив значений меню.
#define MAKE_MENU_VALUES(...) { __VA_ARGS__ }
//! Объявляет переменную и инициалищирует массив значений элемента меню.
#define MENU_VALUES(name, ...) DECLARE_MENU_VALUES_ARRAY(name) = {__VA_ARGS__}

//! Инициализирует пустое значение элемента меню.
#define MAKE_MENU_VALUE_NONE()\
            { .type = MENU_VALUE_TYPE_NONE }
//! Инициализирует строковое значение элемента меню.
#define MAKE_MENU_VALUE_STRING(arg_value_string)\
            { .type = MENU_VALUE_TYPE_STRING, .value_string = arg_value_string }
//! Инициализирует целочисленное значение элемента меню.
#define MAKE_MENU_VALUE_INT(arg_value_int)\
            { .type = MENU_VALUE_TYPE_INT, .value_int = arg_value_int }
//! Инициализирует логическое значение элемента меню.
#define MAKE_MENU_VALUE_BOOL(arg_value_bool)\
            { .type = MENU_VALUE_TYPE_BOOL, .value_bool = arg_value_bool }
//! Инициализирует значение числа с фиксированной запятой элемента меню.
#define MAKE_MENU_VALUE_FIXED(arg_value_fixed)\
            { .type = MENU_VALUE_TYPE_FIXED, .value_fixed = arg_value_fixed }
//! Инициализирует перечислимое значение элемента меню.
#define MAKE_MENU_VALUE_ENUM(arg_current, arg_count, arg_values)\
            { .type = MENU_VALUE_TYPE_ENUM, .value_enum.values = (menu_value_t*)arg_values,\
              .value_enum.count = arg_count, .value_enum.current = arg_current }
//! Инициализирует пользовательское значение элемента меню.
#define MAKE_MENU_VALUE_CUSTOM(arg_value_custom)\
            { .type = MENU_VALUE_TYPE_CUSTOM, .value_custom = arg_value_custom }

//! Обявляет переменную пустого значения элемента меню.
#define MENU_VALUE_NONE(name)\
        DECLARE_MENU_VALUE(name) = MAKE_MENU_VALUE_NONE()
//! Обявляет переменную строкового значения элемента меню.
#define MENU_VALUE_STRING(name, arg_value_string)\
        DECLARE_MENU_VALUE(name) = MAKE_MENU_VALUE_STRING(arg_value_string)
//! Обявляет переменную целочисленного значения элемента меню.
#define MENU_VALUE_INT(name, arg_value_int)\
        DECLARE_MENU_VALUE(name) = MAKE_MENU_VALUE_INT(arg_value_int)
//! Обявляет переменную логического значения элемента меню.
#define MENU_VALUE_BOOL(name, arg_value_bool)\
        DECLARE_MENU_VALUE(name) = MAKE_MENU_VALUE_BOOL(arg_value_bool)
//! Обявляет переменную значения числа с фиксированной запятой элемента меню.
#define MENU_VALUE_FIXED(name, arg_value_fixed)\
        DECLARE_MENU_VALUE(name) = MAKE_MENU_VALUE_FIXED(arg_value_fixed)
//! Обявляет переменную перичислимого значения элемента меню.
#define MENU_VALUE_ENUM(name, arg_current, arg_count, arg_values)\
        DECLARE_MENU_VALUE(name) = MAKE_MENU_VALUE_ENUM(arg_current, arg_count, arg_values)
//! Обявляет переменную перичислимого значения элемента меню списком значений.
#define MENU_VALUE_ENUM_LIST(name, arg_current, arg_count, ...)\
            MENU_VALUES(CONCAT(name, MENU_VALUE_NAME_POSTFIX), __VA_ARGS__ );\
            DECLARE_MENU_VALUE(name) = MAKE_MENU_VALUE_ENUM(arg_current, arg_count, CONCAT(name, MENU_VALUE_NAME_POSTFIX))
//! Обявляет переменную пользовательского значения элемента меню.
#define MENU_VALUE_CUSTOM(name, arg_value_custom)\
        DECLARE_MENU_VALUE(name) = MAKE_MENU_VALUE_CUSTOM(arg_value_custom)


//! Декларирует элемент меню.
#define DECLARE_MENU_ITEM(name) MENU_ITEM_ATTRIBS menu_item_t name
//! Декларирует несколько элементов меню.
#define DECLARE_MENU_ITEMS(...) MENU_ITEM_ATTRIBS menu_item_t __VA_ARGS__

//! Инициализирует элемент меню.
#define MAKE_MENU_ITEM(arg_id, arg_parent, arg_child, arg_prev, arg_next,\
                       arg_text, arg_help, arg_icon_id, arg_value, arg_flags, arg_user_data)\
                      { .id = arg_id, .parent = (menu_item_t*)arg_parent, .child = (menu_item_t*)arg_child,\
                        .prev = (menu_item_t*)arg_prev, .next = (menu_item_t*)arg_next, .text = arg_text,\
                        .help = arg_help, .icon_id = arg_icon_id, .value = (menu_value_t*)arg_value,\
                        .flags = arg_flags .user_data = arg_user_data }
//! Объявляет переменную и инициализирует элемент меню.
#define MENU_ITEM(name, arg_id, arg_parent, arg_child, arg_prev, arg_next,\
                  arg_text, arg_help, arg_icon_id, arg_value, arg_flags, arg_user_data)\
        DECLARE_MENU_ITEM(name) = { .id = arg_id, .parent = (menu_item_t*)arg_parent, .child = (menu_item_t*)arg_child,\
                                    .prev = (menu_item_t*)arg_prev, .next = (menu_item_t*)arg_next, .text = arg_text,\
                                    .help = arg_help, .icon_id = arg_icon_id, .value = (menu_value_t*)arg_value,\
                                    .flags = arg_flags, .user_data = arg_user_data }
//! Объявляет переменную и инициализирует элемент подменю.
#define SUBMENU(name, arg_id, arg_parent, arg_child, arg_prev, arg_next, arg_text, arg_help, arg_icon_id, arg_flags, arg_user_data)\
        DECLARE_MENU_ITEM(name) = { .id = arg_id, .parent = (menu_item_t*)arg_parent, .child = (menu_item_t*)arg_child,\
                                    .prev = (menu_item_t*)arg_prev, .next = (menu_item_t*)arg_next, .text = arg_text,\
                                    .help = arg_help, .icon_id = arg_icon_id, .value = NULL,\
                                    .flags = arg_flags, .user_data = arg_user_data }
//! Объявляет переменную и инициализирует элемент подменю со значением.
#define SUBITEM(name, arg_id, arg_parent, arg_child, arg_prev, arg_next,\
                arg_text, arg_help, arg_icon_id, arg_flags, arg_user_data, arg_value)\
        DECLARE_MENU_VALUE(CONCAT(name, MENU_VALUE_NAME_POSTFIX)) = arg_value;\
        DECLARE_MENU_ITEM(name) = { .id = arg_id, .parent = (menu_item_t*)arg_parent, .child = (menu_item_t*)arg_child,\
                                    .prev = (menu_item_t*)arg_prev, .next = (menu_item_t*)arg_next, .text = arg_text,\
                                    .help = arg_help, .icon_id = arg_icon_id,\
                                    .value = (menu_value_t*)&CONCAT(name, MENU_VALUE_NAME_POSTFIX), .flags = arg_flags,\
                                    .user_data = arg_user_data }
//! Объявляет переменную и инициализирует элемент меню со значением.
#define MENUITEM(name, arg_id, arg_parent, arg_prev, arg_next,\
                 arg_text, arg_help, arg_icon_id, arg_flags, arg_user_data, arg_value)\
        DECLARE_MENU_VALUE(CONCAT(name, MENU_VALUE_NAME_POSTFIX)) = arg_value;\
        DECLARE_MENU_ITEM(name) = { .id = arg_id, .parent = (menu_item_t*)arg_parent, .child = NULL,\
                                    .prev = (menu_item_t*)arg_prev, .next = (menu_item_t*)arg_next, .text = arg_text,\
                                    .help = arg_help, .icon_id = arg_icon_id,\
                                    .value = (menu_value_t*)&CONCAT(name, MENU_VALUE_NAME_POSTFIX), .flags = arg_flags,\
                                    .user_data = arg_user_data }
//! Объявляет переменную и инициализирует элемент меню без значения.
#define MENUACTION(name, arg_id, arg_parent, arg_prev, arg_next,\
                   arg_text, arg_help, arg_icon_id, arg_flags, arg_user_data)\
        DECLARE_MENU_ITEM(name) = { .id = arg_id, .parent = (menu_item_t*)arg_parent, .child = NULL,\
                                    .prev = (menu_item_t*)arg_prev, .next = (menu_item_t*)arg_next, .text = arg_text,\
                                    .help = arg_help, .icon_id = arg_icon_id, .value = NULL,\
                                    .flags = arg_flags, .user_data = arg_user_data }

#endif //MENU_MACRO


#ifdef MENU_DESCR_MACRO

/**
 * Атрибут для переменной массива дескрипторов элемента меню.
 */
#ifndef MENU_DESCRS_ATTRIBS
#define MENU_DESCRS_ATTRIBS
#endif

/**
 * Атрибут для переменной массива элементов меню.
 */
#ifndef MENU_ITEMS_ATTRIBS
#define MENU_ITEMS_ATTRIBS
#endif

//! Инициализирует дескриптор элемента меню по месту объявления.
#define MENU_DESCR(arg_depth, arg_id, arg_text, arg_help, arg_icon_id, arg_flags, arg_user_data, arg_value)\
        { arg_depth, arg_id, arg_text, arg_help, arg_icon_id, arg_flags, arg_user_data, (menu_value_t*)arg_value }

//! Объявляет массив дескрипторов элементов меню.
#define MENU_DESCRS(arg_name)\
        MENU_DESCRS_ATTRIBS menu_descr_t arg_name[] = 

//! Вычисляет число дескрипторов элементов меню в массиве.
#define MENU_DESCRS_COUNT(arg_name) (sizeof(arg_name) / sizeof(menu_descr_t))

//! Объявляет массив элементов меню согласно числу дескрипторов элементов меню.
#define MENU_ITEMS(arg_name, arg_menu_descrs)\
        MENU_ITEMS_ATTRIBS menu_item_t arg_name[MENU_DESCRS_COUNT(arg_menu_descrs)]

//! Вычисляет число элементов меню в массиве.
#define MENU_ITEMS_COUNT(arg_name) (sizeof(arg_name) / sizeof(menu_item_t))

#endif //MENU_DESCR_MACRO


/**
 * Инициализирует меню.
 * @param menu Меню.
 * @param root Корневой элемент меню.
 * @return Код ошибки.
 */
EXTERN err_t menu_init(menu_t* menu, menu_item_t* root);

/**
 * Формирует меню из массива дескрипторов.
 * @param menu Меню.
 * @param items Элементы меню.
 * @param items_count Число элементов меню.
 * @param descrs Дескрипторы.
 * @param descrs_count Число дескрипторов элементов меню.
 * @param append_pred Предикат добавления элемента (может быть NULL).
 * @return Код ошибки.
 */
EXTERN err_t menu_make_from_descrs(menu_t* menu, menu_item_t* items, size_t items_count,
                                           const menu_descr_t* descrs, size_t descrs_count,
                                           bool (*append_pred)(const menu_descr_t* descr));

/**
 * Получает корневой элемент меню.
 * @param menu Меню.
 * @return Корневой элемент меню.
 */
ALWAYS_INLINE static menu_item_t* menu_root(menu_t* menu)
{
    return menu->root;
}

/**
 * Устанавливает корневой элемент меню.
 * @param menu Меню.
 * @param root Корневой элемент меню.
 * @return Код ошибки.
 */
EXTERN err_t menu_set_root(menu_t* menu, menu_item_t* root);

/**
 * Получает текущий элемент меню.
 * @param menu Меню.
 * @return Текущий элемент меню.
 */
ALWAYS_INLINE static menu_item_t* menu_current(menu_t* menu)
{
    return menu->current;
}

/**
 * Устанавливает текущий элемент меню.
 * @param menu Меню.
 * @param current Текущий элемент меню.
 * @return Код ошибки.
 */
EXTERN err_t menu_set_current(menu_t* menu, menu_item_t* current);

/**
 * Сбрасывает текущий элемент меню.
 * @param menu Меню.
 */
EXTERN void menu_reset_current(menu_t* menu);

/**
 * Поднимает текущий элемент меню на уровень выше.
 * @param menu Меню.
 * @return true в случае успеха, иначе false.
 */
EXTERN bool menu_up(menu_t* menu);

/**
 * Поднимает текущий элемент меню на уровень ниже.
 * @param menu Меню.
 * @return true в случае успеха, иначе false.
 */
EXTERN bool menu_down(menu_t* menu);

/**
 * Переходит на следующий элемент меню.
 * @param menu Меню.
 * @return true в случае успеха, иначе false.
 */
EXTERN bool menu_next(menu_t* menu);

/**
 * Переходит на предыдущий элемент меню.
 * @param menu Меню.
 * @return true в случае успеха, иначе false.
 */
EXTERN bool menu_prev(menu_t* menu);

/**
 * Инициализирует элемент меню.
 * @param item Элемент меню.
 * @param text Текст.
 * @return Код ошибки.
 */
EXTERN err_t menu_item_init(menu_item_t* item, const char* text);

/**
 * Инициализирует элемент меню по дескриптору элемента меню.
 * @param item Элемент меню.
 * @param descr Дескриптор элемента меню.
 * @return Код ошибки.
 */
EXTERN err_t menu_item_init_from_descr(menu_item_t* item, const menu_descr_t* descr);

/**
 * @brief Получает число дочерних элементов элемента меню.
 * @param item Элемент меню.
 * @return Число дочерних элементов элемента меню.
 */
EXTERN size_t menu_item_childs_count(menu_item_t* item);

/**
 * @brief Получает элементов меню в списке дочерних элементов меню.
 * @param item Элемент меню.
 * @return Число элементов меню в списке дочерних элементов меню.
 */
EXTERN size_t menu_item_count(menu_item_t* item);

/**
 * @brief Получает дочерний элемент элемента меню с заданным индексом.
 * @param item Элемент меню.
 * @param index Индекс элемента меню.
 * @return Дочерний элемент элемента меню с заданным индексом.
 */
EXTERN menu_item_t* menu_item_child_at(menu_item_t* item, size_t index);

/**
 * @brief Получает следующий элемент элемента меню с заданным индексом.
 * @param item Элемент меню.
 * @param index Индекс элемента меню.
 * @return Следующий элемент элемента меню с заданным индексом.
 */
EXTERN menu_item_t* menu_item_next_at(menu_item_t* item, size_t index);

/**
 * @brief Получает позицию элемента меню в списке дочерних элементов.
 * @param item Элемент меню.
 * @return Позиция элемента меню в списке дочерних элементов.
 */
EXTERN size_t menu_item_pos(menu_item_t* item);

/**
 * Получает идентификатор элемента меню.
 * @param item Элемент меню.
 * @return Идентификатор элемента меню.
 */
ALWAYS_INLINE static menu_id_t menu_item_id(menu_item_t* item)
{
    return item->id;
}

/**
 * Устанавливает идентификатор элемента меню.
 * @param item Элемент меню.
 * @param id Идентификатор элемента меню.
 */
ALWAYS_INLINE static void menu_item_set_id(menu_item_t* item, menu_id_t id)
{
    item->id = id;
}

/**
 * Получает текст элемента меню.
 * @param item Элемент меню.
 * @return Текст элемента меню.
 */
ALWAYS_INLINE static const char* menu_item_text(menu_item_t* item)
{
    return item->text;
}

/**
 * Устанавливает текст элемента меню.
 * @param item Элемент меню.
 * @param text Текст элемента меню.
 */
ALWAYS_INLINE static void menu_item_set_text(menu_item_t* item, const char* text)
{
    item->text = text;
}

/**
 * Получает справку элемента меню.
 * @param item Элемент меню.
 * @return Справка элемента меню.
 */
ALWAYS_INLINE static const char* menu_item_help(menu_item_t* item)
{
    return item->help;
}

/**
 * Устанавливает справку элемента меню.
 * @param item Элемент меню.
 * @param help Справка элемента меню.
 */
ALWAYS_INLINE static void menu_item_set_help(menu_item_t* item, const char* help)
{
    item->help = help;
}

/**
 * Получает идентификатор иконки элемента меню.
 * @param item Элемент меню.
 * @return Идентификатор иконки элемента меню.
 */
ALWAYS_INLINE static menu_icon_id_t menu_item_icon_id(menu_item_t* item)
{
    return item->icon_id;
}

/**
 * Устанавливает идентификатор иконки элемента меню.
 * @param item Элемент меню.
 * @param icon_id Идентификатор иконки элемента меню.
 */
ALWAYS_INLINE static void menu_item_set_icon_id(menu_item_t* item, menu_icon_id_t icon_id)
{
    item->icon_id = icon_id;
}

/**
 * Получает флаги элемента меню.
 * @param item Элемент меню.
 * @return Флаги элемента меню.
 */
ALWAYS_INLINE static menu_flags_t menu_item_flags(menu_item_t* item)
{
    return item->flags;
}

/**
 * Устанавливает флаги элемента меню.
 * @param item Элемент меню.
 * @param flags Флаги элемента меню.
 */
ALWAYS_INLINE static void menu_item_set_flags(menu_item_t* item, menu_flags_t flags)
{
    item->flags = flags;
}

/**
 * Получает пользовательские данные элемента меню.
 * @param item Элемент меню.
 * @return Пользовательские данные элемента меню.
 */
ALWAYS_INLINE static menu_user_data_t menu_item_user_data(menu_item_t* item)
{
    return item->user_data;
}

/**
 * Устанавливает пользовательские данные элемента меню.
 * @param item Элемент меню.
 * @param user_data Пользовательские данные элемента меню.
 */
ALWAYS_INLINE static void menu_item_set_user_data(menu_item_t* item, menu_user_data_t user_data)
{
    item->user_data = user_data;
}

/**
 * Получает предка элемента меню.
 * @param item Элемент меню.
 * @return предок элемента меню.
 */
ALWAYS_INLINE static menu_item_t* menu_item_parent(menu_item_t* item)
{
    return item->parent;
}

/**
 * Устанавливает предка элемента меню.
 * @param item Элемент меню.
 * @param parent предок элемента меню.
 */
ALWAYS_INLINE static void menu_item_set_parent(menu_item_t* item, menu_item_t* parent)
{
    item->parent = parent;
}

/**
 * Получает потомка элемента меню.
 * @param item Элемент меню.
 * @return Потомок элемента меню.
 */
ALWAYS_INLINE static menu_item_t* menu_item_child(menu_item_t* item)
{
    return item->child;
}

/**
 * Устанавливает потомка элемента меню.
 * @param item Элемент меню.
 * @param child Потомок элемента меню.
 */
ALWAYS_INLINE static void menu_item_set_child(menu_item_t* item, menu_item_t* child)
{
    item->child = child;
}

/**
 * Получает предыдущий элемент меню.
 * @param item Элемент меню.
 * @return Предыдущие элемент меню.
 */
ALWAYS_INLINE static menu_item_t* menu_item_prev(menu_item_t* item)
{
    return item->prev;
}

/**
 * Устанавливает предыдущий элемент меню.
 * @param item Элемент меню.
 * @param prev Предыдущий элемент меню.
 */
ALWAYS_INLINE static void menu_item_set_prev(menu_item_t* item, menu_item_t* prev)
{
    item->prev = prev;
}

/**
 * Получает следующий элемент меню.
 * @param item Элемент меню.
 * @return Следующий элемент меню.
 */
ALWAYS_INLINE static menu_item_t* menu_item_next(menu_item_t* item)
{
    return item->next;
}

/**
 * Устанавливает следующий элемент меню.
 * @param item Элемент меню.
 * @param next Следующий элемент меню.
 */
ALWAYS_INLINE static void menu_item_set_next(menu_item_t* item, menu_item_t* next)
{
    item->next = next;
}

/**
 * Получает первый элемент в пределах родителя.
 * @param item Элемент меню.
 * @return Первый элемент меню.
 */
EXTERN menu_item_t* menu_item_first(menu_item_t* item);

/**
 * Получает последний элемент в пределах родителя.
 * @param item Элемент меню.
 * @return Последний элемент меню.
 */
EXTERN menu_item_t* menu_item_last(menu_item_t* item);

/**
 * Связывает два элемента меню.
 * @param prev Предыдущий элемент меню.
 * @param next Следующий элемент меню.
 */
EXTERN void menu_item_link(menu_item_t* prev, menu_item_t* next);

/**
 * Связывает два элемента меню.
 * @param parent Родительский элемент меню.
 * @param child Дочерний элемент меню.
 */
EXTERN void menu_item_link_parent(menu_item_t* parent, menu_item_t* child);

/**
 * Получает значение элемента меню.
 * @param item Элемент меню.
 * @return Значение элемента меню.
 */
ALWAYS_INLINE static menu_value_t* menu_item_value(menu_item_t* item)
{
    return item->value;
}

/**
 * Устанавливает значение элемента меню.
 * @param item Элемент меню.
 * @param value Значение элемента меню.
 */
ALWAYS_INLINE static void menu_item_set_value(menu_item_t* item, menu_value_t* value)
{
    item->value = value;
}

/**
 * Получает флаг наличия значения элемента меню.
 * @param item Элемент меню.
 * @return Флаг наличия значения элемента меню.
 */
ALWAYS_INLINE static bool menu_item_has_value(menu_item_t* item)
{
    return item->value != NULL;
}

/**
 * Инициализирует значение элемента меню.
 * @param value Значение элемента меню.
 * @return Код ошибки.
 */
EXTERN err_t menu_value_init(menu_value_t* value);

/**
 * Получает тип значения элемента меню.
 * @param value Значение элемента меню.
 * @return Тип значения элемента меню.
 */
ALWAYS_INLINE static menu_value_type_t menu_value_type(menu_value_t* value)
{
    return value->type;
}

/**
 * Получает строковое значениче элемента меню.
 * @param value Значение элемента меню.
 * @return Строка - значение элемента меню.
 */
ALWAYS_INLINE static menu_value_string_t menu_value_string(menu_value_t* value)
{
    return value->value_string;
}

/**
 * Устанавливает строковое значение элемента меню.
 * @param value Значение элемента меню.
 * @param val Строка.
 */
ALWAYS_INLINE static void menu_value_set_string(menu_value_t* value, menu_value_string_t val)
{
    value->type = MENU_VALUE_TYPE_STRING;
    value->value_string = val;
}

/**
 * Получает целочисленное значениче элемента меню.
 * @param value Значение элемента меню.
 * @return Число - значение элемента меню.
 */
ALWAYS_INLINE static menu_value_int_t menu_value_int(menu_value_t* value)
{
    return value->value_int;
}

/**
 * Устанавливает целочисленное значение элемента меню.
 * @param value Значение элемента меню.
 * @param val Число.
 */
ALWAYS_INLINE static void menu_value_set_int(menu_value_t* value, menu_value_int_t val)
{
    value->type = MENU_VALUE_TYPE_INT;
    value->value_int = val;
}

/**
 * Получает логическое значениче элемента меню.
 * @param value Значение элемента меню.
 * @return Логическое значение элемента меню.
 */
ALWAYS_INLINE static menu_value_bool_t menu_value_bool(menu_value_t* value)
{
    return value->value_bool;
}

/**
 * Устанавливает логическое значение элемента меню.
 * @param value Значение элемента меню.
 * @param val Логическое значение.
 */
ALWAYS_INLINE static void menu_value_set_bool(menu_value_t* value, menu_value_bool_t val)
{
    value->type = MENU_VALUE_TYPE_BOOL;
    value->value_bool = val;
}

/**
 * Получает значениче с фиксированной запятой элемента меню.
 * @param value Значение элемента меню.
 * @return Значение с фиксированной запятой элемента меню.
 */
ALWAYS_INLINE static menu_value_fixed_t menu_value_fixed(menu_value_t* value)
{
    return value->value_fixed;
}

/**
 * Устанавливает значение с фиксированной запятой элемента меню.
 * @param value Значение элемента меню.
 * @param val Значение с фиксированной запятой.
 */
ALWAYS_INLINE static void menu_value_set_fixed(menu_value_t* value, menu_value_fixed_t val)
{
    value->type = MENU_VALUE_TYPE_FIXED;
    value->value_fixed = val;
}

/**
 * Получает пользовательское значениче элемента меню.
 * @param value Значение элемента меню.
 * @return Пользовательское значениче элемента меню.
 */
ALWAYS_INLINE static menu_value_custom_t menu_value_custom(menu_value_t* value)
{
    return value->value_custom;
}

/**
 * Устанавливает пользовательское значениче элемента меню.
 * @param value Значение элемента меню.
 * @param val Пользовательское значениче.
 */
ALWAYS_INLINE static void menu_value_set_custom(menu_value_t* value, menu_value_custom_t val)
{
    value->type = MENU_VALUE_TYPE_CUSTOM;
    value->value_custom = val;
}

/**
 * Получает списочное значениче элемента меню.
 * @param value Значение элемента меню.
 * @return Списочное значение элемента меню.
 */
ALWAYS_INLINE static menu_value_enum_t* menu_value_enum(menu_value_t* value)
{
    return &value->value_enum;
}

/**
 * Устанавливает списочное значение элемента меню.
 * @param value Значение элемента меню.
 * @param val Списочное значение элемента меню.
 * @return Код ошибки.
 */
EXTERN err_t menu_value_set_enum(menu_value_t* value, menu_value_enum_t* val);

/**
 * Получает текущий индекс списка значений элемента меню.
 * @param value Значение элемента меню.
 * @return Текущей индекс списка значений элемента меню.
 */
ALWAYS_INLINE static size_t menu_value_enum_current(menu_value_t* value)
{
    return value->value_enum.current;
}

/**
 * Устанавливает текущий индекс списка значений элемента меню.
 * @param value Значение элемента меню.
 * @param current Текущей индекс списка значений элемента меню.
 */
ALWAYS_INLINE static void menu_value_enum_set_current(menu_value_t* value, size_t current)
{
    if(current < value->value_enum.count) value->value_enum.current = current;
}

/**
 * Получает число значений списка значений элемента меню.
 * @param value Значение элемента меню.
 * @return Число значений списка значений элемента меню.
 */
ALWAYS_INLINE static size_t menu_value_enum_count(menu_value_t* value)
{
    return value->value_enum.count;
}

/**
 * Устанавливает число значений списка значений элемента меню.
 * @param value Значение элемента меню.
 * @param count Число значений списка значений элемента меню.
 */
EXTERN err_t menu_value_enum_set_count(menu_value_t* value, size_t count);

/**
 * Получает значения списка значений элемента меню.
 * @param value Значение элемента меню.
 * @return Значения списка значений элемента меню.
 */
ALWAYS_INLINE static menu_value_t* menu_value_enum_values(menu_value_t* value)
{
    return value->value_enum.values;
}

/**
 * Устанавливает значения списка значений элемента меню.
 * @param value Значение элемента меню.
 * @param values Значения списка значений элемента меню.
 */
EXTERN err_t menu_value_enum_set_values(menu_value_t* value, menu_value_t* values);

/**
 * Получает текущие значение списка значений элемента меню.
 * @param value Значение элемента меню.
 * @return Текущее значение списка значений элемента меню.
 */
EXTERN menu_value_t* menu_value_enum_current_value(menu_value_t* value);


/*
 * Пример.
 *
 * Создание такого меню:
 * 
 * |-item1
 * |-item2
 * |-item3
 *     |-item4
 *     |-item5
 * 

// Объявление элементов меню.
DECLARE_MENU_ITEMS(item1, item2, item3, item4, item5);

// Создание меню.
MENU(menu0, &item1);

// Создание списка значений для значения элемента 5.
MENU_VALUES(menu_values_en_dis, MAKE_MENU_VALUE_STRING("Enabled"), MAKE_MENU_VALUE_STRING("Disabled"));

// Создание самого меню.

//        Имя  ID  Предок  Предыдущий Следующий Текст      Справка Иконка Флаги Польз.данные Значения
MENUITEM(item1, 0, NULL,   NULL,      &item2,   "Menu0_0", "Item", 0,     0,    0,           MAKE_MENU_VALUE_STRING("Text"));
MENUITEM(item2, 0, NULL,   &item1,    &item3,   "Menu0_1", "Item", 0,     0,    0,           MAKE_MENU_VALUE_INT(123));

//         Имя  ID  Предок  Потомок Предыдущий Следующий    Текст      Справка Иконка Флаги Польз.данные
SUBMENU (item3, 0,  NULL,   &item4, &item2,    NULL,        "Menu0_2", "Menu", 0,     0,    0);

//         Имя  ID Предок  Предыдущий Следующий Текст      Справка Иконка Флаги Польз.данные Значения
MENUITEM(item4, 0, &item3, NULL,      &item5,   "Menu1_0", "Item", 0,     0,    0,           MAKE_MENU_VALUE_BOOL(true));
MENUITEM(item5, 0, &item3, &item4,    NULL,     "Menu1_1", "Item", 0,     0,    0,           MAKE_MENU_VALUE_ENUM(0, 2, menu_values_en_dis));

*/

/*
 * Пример 2.
 * 
 * Создание вышеприведённого меню используя дескрипторы.
 * 

// Объявление меню.
menu_t test_menu;

// Объявление массива дескрипторов.
MENU_DESCRS(test_menu_descrs) {
    //         Уровень ID Текст      Справка Иконка Флаги Польз.данные Значения
    MENU_DESCR(0,      0, "Menu0_0", "Item", 0,     0,    0,           0),
    MENU_DESCR(0,      0, "Menu0_1", "Item", 0,     0,    0,           0),
    MENU_DESCR(0,      0, "Menu0_2", "Menu", 0,     0,    0,           0),
        MENU_DESCR(1,  0, "Menu1_0", "Item", 0,     0,    0,           0),
        MENU_DESCR(1,  0, "Menu1_1", "Item", 0,     0,    0,           0)
};

// Объявление массива элементов меню.
MENU_ITEMS(test_menu_items, test_menu_descrs);

// В функции инициализации меню менобходимо сформировать
// меню посредством вызова соответствующей функции.

menu_make_from_descrs(&test_menu, test_menu_items, MENU_ITEMS_COUNT(test_menu_items),
                                  test_menu_descrs, MENU_DESCRS_COUNT(test_menu_descrs));

 */

#endif	/* MENU_H */
