/****************************************************************************
 Copyright (c) 2015 Victor Komarov
 Copyright (c) 2015 Jeff Wang

 https://github.com/fnz
 https://github.com/summerinsects
 
 Permission is hereby granted, free of charge, to any person obtaining a copy
 of this software and associated documentation files (the "Software"), to deal
 in the Software without restriction, including without limitation the rights
 to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 copies of the Software, and to permit persons to whom the Software is
 furnished to do so, subject to the following conditions:
 The above copyright notice and this permission notice shall be included in
 all copies or substantial portions of the Software.
 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 THE SOFTWARE.
 ****************************************************************************/

#pragma once

#if CC_TARGET_PLATFORM == CC_PLATFORM_ANDROID

#include <jni.h>
#include "jni/JniHelper.h"
#include "platform/CCPlatformConfig.h"
#include "base/ccUTF8.h"

namespace EasyJNIDetail {
    struct LocalRefWrapper {
        LocalRefWrapper(JNIEnv* env, jobject obj) : _env(env), _obj(obj) { }
        ~LocalRefWrapper() { _env->DeleteLocalRef(_obj); }
        // FIXME: copy constructor, move constructor
    private:
        JNIEnv* _env;
        jobject _obj;
    };

    //
    // ArgumentWrapper
    //
    template <class T> class ArgumentWrapper {
        T _arg;
    public:
        ArgumentWrapper(JNIEnv*, T arg) : _arg(arg) { }
        inline T get() const { return _arg; };
    };

    template <> class ArgumentWrapper<const char*> {
        JNIEnv* _env;
        jstring _str;

        ArgumentWrapper(const ArgumentWrapper&) = delete;
        ArgumentWrapper(ArgumentWrapper&&) = delete;
        ArgumentWrapper& operator=(const ArgumentWrapper&) = delete;
        ArgumentWrapper& operator=(ArgumentWrapper&&) = delete;

        inline void set(const char *str) {
            _str = cocos2d::StringUtils::newStringUTFJNI(_env, str ? str : "");
        }

    public:
        ~ArgumentWrapper() { _env->DeleteLocalRef(_str); }
        ArgumentWrapper(JNIEnv* env, const char* str) : _env(env) { set(str); }
        ArgumentWrapper(JNIEnv* env, const std::string& str) : _env(env) { set(str.c_str()); }

        inline jstring get() const { return _str; };
    };

    //
    // ArgumentTypeConverter
    //
    template <class T> struct ArgumentTypeConverter {
        typedef T Type;
    };

    template <> struct ArgumentTypeConverter<std::string> {
        typedef const char* Type;
    };

    template <size_t N> struct ArgumentTypeConverter<char [N]> : ArgumentTypeConverter<std::string> { };
    template <size_t N> struct ArgumentTypeConverter<const char [N]> : ArgumentTypeConverter<std::string> { };

    template <class T> struct ArgumentTypeConverter<const T> :ArgumentTypeConverter<T> { };
    template <class T> struct ArgumentTypeConverter<T&> :ArgumentTypeConverter<T> { };
    template <class T> struct ArgumentTypeConverter<const T&> :ArgumentTypeConverter<T> { };
    template <class T> struct ArgumentTypeConverter<T&&> :ArgumentTypeConverter<T> { };

    //
    // CharSequence
    //
    template <char... Chars>
    struct CharSequence {
        static const char value[sizeof...(Chars) + 1];
    };

    template <char... Chars>
    const char CharSequence<Chars...>::value[sizeof...(Chars) + 1] = {
        Chars...,
    };

    //
    // SequenceConcatenator
    //
    template <class Seq, class... Seqs>
    struct SequenceConcatenator;

    template <char... Chars>
    struct SequenceConcatenator<CharSequence<Chars...> > {
        typedef CharSequence<Chars...> Result;
    };

    template <char... Chars1, char...Chars2>
    struct SequenceConcatenator<CharSequence<Chars1...>, CharSequence<Chars2...> > {
        typedef CharSequence<Chars1..., Chars2...> Result;
    };

    template <char... Chars1, char...Chars2, class ... Seq>
    struct SequenceConcatenator<CharSequence<Chars1...>, CharSequence<Chars2...>, Seq...> {
        typedef typename SequenceConcatenator<CharSequence<Chars1..., Chars2...>, Seq...>::Result Result;
    };

    //
    // JNISignature
    //
    template <class T, class... Ts> struct JNISignature {
        typedef typename SequenceConcatenator<typename JNISignature<T>::Sequence, typename JNISignature<Ts...>::Sequence>::Result Sequence;
    };

    template <> struct JNISignature<bool> {
        typedef CharSequence<'Z'> Sequence;
    };

    template <> struct JNISignature<uint8_t> {
        typedef CharSequence<'B'> Sequence;
    };

    template <> struct JNISignature<uint16_t> {
        typedef CharSequence<'C'> Sequence;
    };

    template <> struct JNISignature<short> {
        typedef CharSequence<'S'> Sequence;
    };

    template <> struct JNISignature<int> {
        typedef CharSequence<'I'> Sequence;
    };

    template <> struct JNISignature<long> {
        typedef CharSequence<'J'> Sequence;
    };

    template <> struct JNISignature<int64_t> {
        typedef CharSequence<'J'> Sequence;
    };

    template <> struct JNISignature<float> {
        typedef CharSequence<'F'> Sequence;
    };

    template <> struct JNISignature<double> {
        typedef CharSequence<'D'> Sequence;
    };

    template <> struct JNISignature<void> {
        typedef CharSequence<'V'> Sequence;
    };

    template <> struct JNISignature<char*> {
        typedef CharSequence<'L', 'j', 'a', 'v', 'a', '/', 'l', 'a', 'n', 'g', '/', 'S', 't', 'r', 'i', 'n', 'g', ';'> Sequence;
    };
    template <> struct JNISignature<const char*> : JNISignature<char*> { };
    template <size_t N> struct JNISignature<char [N]> : JNISignature<char*> { };
    template <size_t N> struct JNISignature<const char [N]> : JNISignature<char*> { };

    template <> struct JNISignature<std::string> : JNISignature<char*> { };

    template <class T> struct JNISignature<std::vector<T> > {
        typedef typename SequenceConcatenator<CharSequence<'['>, typename JNISignature<T>::Sequence>::Result Sequence;
    };

    template <class T> struct JNISignature<const T> : JNISignature<T> { };
    template <class T> struct JNISignature<T&> : JNISignature<T> { };
    template <class T> struct JNISignature<const T&> : JNISignature<T> { };
    template <class T> struct JNISignature<T&&> : JNISignature<T> { };

    //
    // SignatureGetter
    //
    template <class T> struct SignatureGetter;

    template <class Ret, class... Args> struct SignatureGetter<Ret (Args...)> {
        typedef typename SequenceConcatenator<CharSequence<'('>,
            typename JNISignature<Args...>::Sequence,
            CharSequence<')'>,
            typename JNISignature<Ret>::Sequence>::Result SignatureSequence;
    };

    template<class Ret> struct SignatureGetter<Ret ()> {
        typedef typename SequenceConcatenator<CharSequence<'(', ')'>,
            typename JNISignature<Ret>::Sequence>::Result SignatureSequence;
    };

    //
    // MethodInvokerImpl
    //
    template <class T> struct MethodInvokerImpl { };

    template <> struct MethodInvokerImpl<bool> {
        static bool staticInvoke(JNIEnv* env, jclass clazz, jmethodID methodID, va_list args) {
            return JNI_TRUE == env->CallStaticBooleanMethodV(clazz, methodID, args);
        }
    };

    template <> struct MethodInvokerImpl<uint8_t> {
        static uint8_t staticInvoke(JNIEnv* env, jclass clazz, jmethodID methodID, va_list args) {
            return env->CallStaticByteMethodV(clazz, methodID, args);
        }
    };

    template <> struct MethodInvokerImpl<uint16_t> {
        static uint16_t staticInvoke(JNIEnv* env, jclass clazz, jmethodID methodID, va_list args) {
            return env->CallStaticCharMethodV(clazz, methodID, args);
        }
    };

    template <> struct MethodInvokerImpl<short> {
        static short staticInvoke(JNIEnv* env, jclass clazz, jmethodID methodID, va_list args) {
            return env->CallStaticShortMethodV(clazz, methodID, args);
        }
    };

    template <> struct MethodInvokerImpl<int> {
        static int staticInvoke(JNIEnv* env, jclass clazz, jmethodID methodID, va_list args) {
            return env->CallStaticIntMethodV(clazz, methodID, args);
        }
    };

    template <> struct MethodInvokerImpl<long> {
        static long staticInvoke(JNIEnv* env, jclass clazz, jmethodID methodID, va_list args) {
            return static_cast<long>(env->CallStaticLongMethodV(clazz, methodID, args));
        }
    };

    template <> struct MethodInvokerImpl<int64_t> {
        static int64_t staticInvoke(JNIEnv* env, jclass clazz, jmethodID methodID, va_list args) {
            return env->CallStaticLongMethodV(clazz, methodID, args);
        }
    };

    template <> struct MethodInvokerImpl<float> {
        static float staticInvoke(JNIEnv* env, jclass clazz, jmethodID methodID, va_list args) {
            return env->CallStaticFloatMethodV(clazz, methodID, args);
        }
    };

    template <> struct MethodInvokerImpl<double> {
        static double staticInvoke(JNIEnv* env, jclass clazz, jmethodID methodID, va_list args) {
            return env->CallStaticDoubleMethodV(clazz, methodID, args);
        }
    };

    template <> struct MethodInvokerImpl<void> {
        static void staticInvoke(JNIEnv* env, jclass clazz, jmethodID methodID, va_list args) {
            env->CallStaticVoidMethodV(clazz, methodID, args);
        }
    };

    template <> struct MethodInvokerImpl<std::string> {
        static std::string staticInvoke(JNIEnv* env, jclass clazz, jmethodID methodID, va_list args) {
            jstring jret = (jstring)env->CallStaticObjectMethodV(clazz, methodID, args);
            LocalRefWrapper temp(env, jret);
            return cocos2d::StringUtils::getStringUTFCharsJNI(env, jret);
        }
    };

    template <> struct MethodInvokerImpl<const char *> : MethodInvokerImpl<std::string> { };

    //
    // MethodInvoker
    //
    template <class Ret>
    struct MethodInvoker {
        static Ret staticInvoke(JNIEnv* env, jclass clazz, jmethodID methodID, ...) {
            va_list args;
            va_start(args, methodID);
            Ret ret = MethodInvokerImpl<Ret>::staticInvoke(env, clazz, methodID, args);
            va_end(args);
            return ret;
        }
    };

    template <>
    struct MethodInvoker<void> {
        static void staticInvoke(JNIEnv* env, jclass clazz, jmethodID methodID, ...) {
            va_list args;
            va_start(args, methodID);
            MethodInvokerImpl<void>::staticInvoke(env, clazz, methodID, args);
            va_end(args);
        }
    };
}

#define LOG_TAG "EasyJNI"
#define LOGE(...)  __android_log_print(ANDROID_LOG_ERROR,LOG_TAG,__VA_ARGS__)

class EasyJNI {
public:
    template <typename Ret, typename... Args>
    static Ret callStaticMethod(const char* className, const char* methodName, const Args& ...args) {
        typedef typename EasyJNIDetail::SignatureGetter<Ret (Args...)>::SignatureSequence SignatureSequence;
        const char* signature = SignatureSequence::value;

        cocos2d::JniMethodInfo t;
        if (cocos2d::JniHelper::getStaticMethodInfo(t, className, methodName, signature)) {
            EasyJNIDetail::LocalRefWrapper clazz(t.env, t.classID);
            return EasyJNIDetail::MethodInvoker<Ret>::staticInvoke(t.env, t.classID, t.methodID,
                EasyJNIDetail::ArgumentWrapper<typename EasyJNIDetail::ArgumentTypeConverter<Args>::Type>(t.env, args).get()...);
        }
        else {
            reportError(className, methodName, signature);
            return Ret();
        }
    }

private:
    static void reportError(const char* className, const char* methodName, const char* signature) {
        LOGE("Failed to find static java method. Class name: %s, method name: %s, signature: %s ",  className, methodName, signature);
    }
};

#endif