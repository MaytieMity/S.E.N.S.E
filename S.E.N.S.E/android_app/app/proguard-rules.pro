# Keep JavascriptInterface methods
-keepclassmembers class com.sense.monitor.MainActivity$AndroidBridge {
    @android.webkit.JavascriptInterface <methods>;
}
