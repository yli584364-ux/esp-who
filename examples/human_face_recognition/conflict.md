可以，这里给你一段**可直接写进 README 的总结说明**，已经整理成工程级别的“兼容性修复说明”。

---

# ⚠️ ESP-WHO + ESP-DL 兼容性问题说明（ESP32-P4）

## 📌 问题背景

在本项目中引入：

* `espressif/hand_detect`
* `espressif/esp-dl`（自动依赖）

后，会出现编译错误，原因是：

> **ESP-WHO 当前代码基于旧版 esp-dl API，而 Component Manager 拉取的是新版 esp-dl（API 已变化）**

---

## 🔥 典型报错

### 1️⃣ 像素类型错误

```cpp
DL_IMAGE_PIX_TYPE_RGB565 is not a member of dl::image
```

---

### 2️⃣ 图像大小函数错误

```cpp
get_img_byte_size is not a member of dl::image
```

---

### 3️⃣ Transformer 接口错误

```cpp
ImageTransformer has no member named set_caps
```

---

## 🧠 根本原因

新版 `esp-dl` 已进行 API 重构：

| 功能          | 旧 API                 | 新 API      |
| ----------- | --------------------- | ---------- |
| 像素格式        | `RGB565`              | `RGB565LE` |
| 图像大小        | `get_img_byte_size()` | ❌ 删除       |
| Transformer | `set_caps()`          | ❌ 删除       |

---

## ✅ 解决方法（兼容性修改）

### 🔧 修改 1：像素类型

文件：

```text
components/who_peripherals/who_cam/who_cam_define.hpp
```

替换：

```cpp
DL_IMAGE_PIX_TYPE_RGB565
```

为：

```cpp
DL_IMAGE_PIX_TYPE_RGB565LE
```

---

### 🔧 修改 2：图像大小计算

文件：

```text
components/who_peripherals/who_cam/who_cam_define.hpp
components/who_frame_cap/who_frame_cap_node.cpp
```

替换：

```cpp
dl::image::get_img_byte_size(img)
```

为：

```cpp
img.width * img.height * dl::image::get_pix_byte_size(img.pix_type)
```

---

### 🔧 修改 3：删除 set_caps

文件：

```text
main/boundary_monitor_app.cpp
```

删除：

```cpp
m_image_transformer.set_caps(m_caps);
```

---

## ⚠️ 注意事项

* 新版 `ImageTransformer` 已不需要 `set_caps()`，参数通常自动处理
* 若出现图像异常（尺寸/颜色），需手动检查：

  * 输入分辨率
  * 像素格式
  * resize逻辑

---

## 🚀 推荐开发路径

### ✔ 当前方案（兼容 patch）

```text
esp-who pipeline + hand_detect + 修改 API
```

适合：

* 快速跑通 demo
* 保留现有 pipeline

---

### ✔ 推荐长期方案

```text
camera → frame → hand_detect → ROI判断 → 显示/报警
```

👉 直接绕过：

* who_detect
* who_frame_cap（部分）
* 旧 API 依赖

---

## 📦 额外说明

本问题属于：

> **组件版本不一致导致的 API 断裂**

在 ESP-IDF Component Manager 项目中较常见，建议：

* 固定组件版本（可选）
* 或逐步适配新版 API（推荐）

---

## 🧩 总结一句话

> 当前问题不是代码写错，而是 **esp-who（旧）与 esp-dl（新）接口不匹配，需要做兼容性修复**

---

如果你后面还要写 README 的“架构说明”或“pipeline 图”，我也可以帮你补一版更专业的版本。
