
# Real-Time Spatial Reasoning by Mobile Robots for Reconstruction and Navigation in Dynamic LiDAR Scenes

<!-- <a href="https://pytorch.org/"><img alt="PyTorch" src="https://img.shields.io/badge/PyTorch-EE4C2C?style=for-the-badge&logo=pytorch&logoColor=white"></a>
<a href="https://github.com/huggingface/accelerate"><img alt="Accelerate" src="https://img.shields.io/badge/Accelerate-ffd21e?style=for-the-badge&logo=Accelerate&logoColor=white"></a>
<a href="https://wandb.ai/site"><img alt="WandB" src="https://img.shields.io/badge/Weights_&_Biases-FFBE00?style=for-the-badge&logo=WeightsAndBiases&logoColor=white"></a> -->

**This repository is the official repository of the paper, *Real-Time Spatial Reasoning by Mobile Robots for Reconstruction and Navigation in Dynamic LiDAR Scenes*.**

[Pengdi Huang](https://github.com/alualu628628),
[Mingyang Wang](),
[Huan Tian](https://github.com/LogicT5),
[Minglun Gong](https://www.uoguelph.ca/computing/people/minglun-gong),
[Hao (Richard) Zhang](https://www.cs.sfu.ca/~haoz/),
[Hui Huang](https://vcc.tech/~huihuang)

[VCC](https://vcc.tech/), 
[CSSE](https://csse.szu.edu.cn/),
[Shenzhen University](https://www.szu.edu.cn/)


<!-- ### [Project Page](https://vcc.tech/research/2025/CLRWire) | [Paper (ArXiv)](https://www.arxiv.org/abs/2504.19174) -->


<img src='assets/overview.png'/>

## Demos
<!-- <video src="assets/RealRecon_demo_1.mp4" controls="controls" style="min-width: 640px; min-height: 360px;"></video> -->

<!-- <video src="assets/RealRecon_demo_2.mp4" controls="controls" style="min-width: 640px; min-height: 360px;"></video> -->

<!-- <video src="assets/RealRecon_demo_3.mp4" controls="controls" style="min-width: 640px; min-height: 360px;"></video> -->

<!-- 把下面这段放到你的 .md 文件里 -->
<style>
  .container {
    display: flex;
    align-items: flex-start;
    gap: 0.5rem;
  }
  .video-container {
    min-width: 640px;
    min-height: 360px;
    background: #000;
  }
  .video-container video {
    width: 100%;
    height: 100%;
    object-fit: cover;
  }
  ul#videolist {
    list-style: none;
    padding: 0;
    margin: 0;
  }
  ul#videolist li {
    padding: 0.5rem 0.1rem;
    cursor: pointer;
    border-radius: 0.1rem;
  }
  ul#videolist li.active {
    background: #eef;
    font-weight: bold;
  }
  ul#videolist li:hover {
    background: #f5f5f5;
  }
</style>

<div class="container">
  <!-- 视频播放区 -->
  <div class="video-container">
    <video id="player" src="assets/RealRecon_demo_1.mp4" controls muted playsinline></video>
  </div>

  <!-- 点击切换播放的视频列表 -->
  <ul id="videolist">
    <li data-src="https://youtu.be/t4Y_ba83do0" class="active">Demo_1</li>
    <li data-src="https://youtu.be/bxgGxpoeJQk">Demo_2</li>
    <li data-src="https://youtu.be/fvawxgCCN40">Demo_3</li>
  </ul>
</div>

<script>
  const player = document.getElementById('player');
  const list   = document.getElementById('videolist');

  list.addEventListener('click', e => {
    if (e.target.tagName.toLowerCase() !== 'li') return;
    // 切换列表高亮
    list.querySelectorAll('li').forEach(li => li.classList.remove('active'));
    e.target.classList.add('active');
    // 切换视频源并播放
    const src = e.target.getAttribute('data-src');
    if (player.src.endsWith(src)) return; // 已是当前视频则不重复加载
    player.src = src;
    player.play();
  });
</script>



## Installation
ROS Noetic. Follow [ROS Installation](https://wiki.ros.org/noetic/Installation/Ubuntu)


## Download data and checkpoints


## Usage



## :notebook_with_decorative_cover: Citation

If you find our work useful for your research, please consider citing the following papers :



## :email: Contact
