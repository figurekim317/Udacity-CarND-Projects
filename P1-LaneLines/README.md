# **Finding Lane Lines on the Road** 

[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

**Finding Lane Lines on the Road**

The goals / steps of this project are the following:
* Make a pipeline that finds lane lines on the road
* Reflect on your work in a written report


[//]: # (Image References)
[pipeline1]: ./resources/pipeline1.PNG "pipeline1"
[pipeline2]: ./resources/pipeline2.PNG "pipeline2"


---

### Reflection

### 1. Describe your pipeline. As part of the description, explain how you modified the draw_lines() function.

My pipeline consisted of 5 steps. First, I converted the images to grayscale, then second I blurred the image. Third, I detected the edges with canny edge detection methods. Fourth, cropped the region that I'm interest only. After that I applyed the hough transform and drawed the lines.

I made the simple tools that visualizes the parameter interactively so that I can find the optimal parameters easily. Here is the picture. The parameters are set quite strict than the sample image so that it reduces the noise but less sensitive.

![parameter image][pipeline1]

In order to draw a single line on the left and right lanes, I modified the draw_lines() function. First, it calculates average endpoint coordinates of lines. It classifies left and right lane by its slope. After that, it drops the noises (especially for the yellow line video) if the slope is less than some specific tangent value.
![parameter image][pipeline2]


### 2. Identify potential shortcomings with your current pipeline

One potential shortcoming would be what would happen when it's applied to the different environment. When I applied this to the third video, canny edge methods catched the lane lines well. However it also catched other features of the road such as change in pavement, the walls and else.

Another shortcoming could be that this parameters are not adaptive. When the brightness or climate changes the optimal parameters would not be the same, but my pipeline can't correct it by itself.


### 3. Suggest possible improvements to your pipeline

A possible improvement would be the adaptivity to change in conditions including brightness and weather.

Another potential improvement could be the ability to distinguish the lane lines from other features/objects.
 
