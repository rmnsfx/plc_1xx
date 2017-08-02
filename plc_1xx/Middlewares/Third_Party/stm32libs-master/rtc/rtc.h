/**
 * @file rtc.h
 * Библиотека для работы со встроенными часами реального времени.
 */

#ifndef RTC_H
#define	RTC_H

#include <stm32f10x.h>
#include <stdint.h>
#include <stdbool.h>
#include <time.h>
#include <sys/time.h>
#include "errors/errors.h"
#include "defs/defs.h"


//! Функция обратного вызова при тике часов на секунду.
typedef void (*rtc_second_callback_t)(void);

//! Функция обратного вызова при достижении времени будильника.
typedef void (*rtc_alarm_callback_t)(void);

//! Функция обратного вызова для получения микросекунд.
typedef suseconds_t (*rtc_get_usec_callback_t)(void);

/**
 * Инициализирует часы реального времени.
 * @return Код ошибки.
 */
EXTERN err_t rtc_init(void);

/**
 * Получает состояние часов реального времени.
 * @return Состояние часов реального времени.
 */
EXTERN FunctionalState rtc_state(void);

/**
 * Устанавливает начальные значения предделителя и времени.
 * @param prescaler Предделитель.
 * @param time Время в секундах.
 */
EXTERN void rtc_setup(uint32_t prescaler, const time_t* time);

/**
 * Получает время в секундах.
 * @param time Опциональная переменная для записи времени.
 * @return Время в секундах.
 */
EXTERN time_t rtc_time(time_t* time);

/**
 * Устанавливает время в секундах.
 * @param time Время в секундах, при NULL устанавливает 0 секунд.
 */
EXTERN void rtc_set_time(const time_t* time);

/**
 * Получает время сигнализации в секундах.
 * @param time Опциональная переменная для записи времени сигнализации.
 * @return Время сигнализации в секундах.
 */
EXTERN time_t rtc_alarm(time_t* time);

/**
 * Устанавливает время сигнализации в секундах.
 * @param time Время сигнализации в секундах, при NULL устанавливает 0 секунд.
 */
EXTERN void rtc_set_alarm(const time_t* time);

/**
 * Получает предделитель.
 * @return Предделитель.
 */
EXTERN uint32_t rtc_prescaler(void);

/**
 * Устанавливает предделитель.
 * @param prescaler предделитель.
 */
EXTERN void rtc_set_prescaler(uint32_t prescaler);

/**
 * Ждёт синхронизации часов.
 */
EXTERN void rtc_synchronize(void);

/**
 * Очищает флаг синхронизации часов.
 */
EXTERN void rtc_clear_sync(void);

/**
 * Ждёт установки флага синхронизации часов.
 */
EXTERN void rtc_wait_sync(void);

/**
 * Получает флаг синхронизации часов.
 * @return Флаг синхронизации часов.
 */
EXTERN bool rtc_synchronized(void);

/**
 * Обработчик ежесекундного прерывания.
 */
EXTERN void rtc_interrupt_handler(void);

/**
 * Обработчик прерывания сигнализации.
 */
EXTERN void rtc_alarm_interrupt_handler(void);

/**
 * Получает текущий секундный каллбэк.
 * @return Секундный каллбэк.
 */
EXTERN rtc_second_callback_t rtc_second_callback(void);

/**
 * Устанавливает секундный каллбэк.
 * Разрешает секундное прерывание, если каллбэк не равен NULL,
 * иначе запрещает его.
 * @param callback Секундный каллбэк.
 */
EXTERN void rtc_set_second_callback(rtc_second_callback_t callback);

/**
 * Получает текущий каллбэк сигнализации.
 * @return Каллбэк сигнализации.
 */
EXTERN rtc_alarm_callback_t rtc_alarm_callback(void);

/**
 * Устанавливает каллбэк сигнализации.
 * Разрешает прерывание сигнализации, если каллбэк не равен NULL,
 * иначе запрещает его.
 * @param callback Каллбэк сигнализации.
 */
EXTERN void rtc_set_alarm_callback(rtc_alarm_callback_t callback);

/**
 * Получает текущий каллбэк получения микросекунд.
 * @return Каллбэк получения микросекунд.
 */
EXTERN rtc_get_usec_callback_t rtc_get_usec_callback(void);

/**
 * Устанавливает каллбэк получения микросекунд.
 * @param callback Каллбэк получения микросекунд.
 */
EXTERN void rtc_set_get_usec_callback(rtc_get_usec_callback_t callback);

#ifdef RTC_TIMEOFDAY

/**
 * Получает текущее время.
 * @param tv Текущее время.
 * @param tz Текущая временная зона.
 * @return 0 при успешном завершении работы, -1 в случае ошибки.
 */
EXTERN int _gettimeofday(struct timeval *tv, struct timezone *tz);

/**
 * Устанавливает текущее время.
 * @param tv Текущее время.
 * @param tz Текущая временная зона.
 * @return 0 при успешном завершении работы, -1 в случае ошибки.
 */
EXTERN int settimeofday(const struct timeval *tv, const struct timezone *tz);

#endif

#endif	/* RTC_H */
