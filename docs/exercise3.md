# Exercise 3 Report

## TL;DR

Text for tldr will go here

## Part 1
---
### Computer Vision

<br>

Below you can see a video of the duckiebot detecting the apriltags. When this first started working, it was incredibly cool to see!

**video here!**

<br>

**What does the april tag library return to you for determining its position?**

The april tag library returns an AprilTag object for us! This is incredibly helpful since we are able to use commands like `apriltag.corners` to help draw the bounding box of the image. 

<br>

**Which directions do the X, Y, Z values of your detection increase / decrease?**

![](../docs/assets/images_ex3/frame.png)
<figcaption align = "center"><b>Fig.1: Diagram to find tick rotation</b></figcaption>

<br>

As stated in the above picture, april tag detections will increase on the *Z* and *X* axis as they move in the positive direction. April tag detections will decrease as they move in the negative direction on the *Y* axis. 

<br>

**What frame orientation does the april tag use?**

<u>not sure about this question</u>

The frame orientation can be found using the right hand rule. Utilizing the RHR, we are able to see a frame orientation similar to Fig.1 

<br>

**Why are detections from far away prone to error?**

Detections from far away can be prone to error due to a few reasons:

- The angle of the camera: Since the camera is tilted down, some distortion can occur when trying to detect tags
- The field of view: With a wide field of view an image can look distorted when compared to a smaller FOV
- The physical camera: The physical camera equipped on the duckiebot is not the best quality. It also has a slight fisheye lens which can cause added distortion


<br>

**Why may you want to limit the rate of detections?**

Since our duckiebot is not equipped with very much onboard memory, it can get easily overloaded by trying to detect many objects in a small period of time. By limiting the number of detections, we can increase our operating efficeny and also speed up subsequent detections. 

<br>

## Part 2
---
### Lane Following

<br>

Below you can see a video of the duckiebot utilizing the PID controller and lane following <u>American</u> driver style.

**video here!**

<br>

Below you can see a video of the duckiebot utilizing the PID controller and lane following <u>English</u> driver style.

**video here!**

<br>


**What is the error for your PID controller?**

answer

<br>

**If your proportional controller did not work well alone, what could have caused this?**

answer

<br>

**Does the D term help your controller logic? Why or why not?**

answer

<br>

**(Optional) Why or why not was the I term useful for your robot?**

answer

<br>

## Part 3
---
### Localization using Sensor Fusion

<br>

all of the stuff goes here


<br>

## References
---
[Lab Manual](https://eclass.srv.ualberta.ca/pluginfile.php/9276727/mod_resource/content/3/Exercise%203.pdf)

[Classical Robotics Architectures using Duckietownown](https://docsduckietown.org/daffy/duckietown-classical-roboticsduckietown-classical-robotics-ready-tablet.pdf)

[Apriltag with Python](https://pyimagesearch.com/2020/11/02/apriltag-with-python/)

[dt-core Library](https://github.com/duckietown/dt-core)

[CMPUT 412 Cheat Sheet](https://docs.google.com/document/d/1bQfkR_tmwctFozEZlZkmojBZHkegscJPJVuw-IEXwI4/edit#)

[Duckietown Docs](https://docs.duckietown.org/daffy/)

[Markdown Syntax](https://bookdown.org/yihui/rmarkdown/markdown-syntax.html#)
