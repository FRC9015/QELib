package com.qelib.jni;

import org.junit.jupiter.api.Test;

import com.qelib.jni.VendorJNI;

public class VendorJNITest {
  @Test
  void jniLinkTest() {
    // Test to verify that the JNI test link works correctly.
    VendorJNI.initialize();
  }
}