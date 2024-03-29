# Multi_AGV_Planner
Just about my master works, including global planner, trajectory planner, decision algorithm and so on. 

ST_Astar + DP + QP

### Slow down to avoid obstacles
<div align=center>
<img src=https://github.com/Qin1143/Multi_AGV_Planner/blob/main/Figures/low-speed02.png>
</div>
<div align=center>
<img src=https://github.com/Qin1143/Multi_AGV_Planner/blob/main/Figures/low-speed02.gif>
</div>

### Accelerate over obstacles
<div align=center>
<img src=https://github.com/Qin1143/Multi_AGV_Planner/blob/main/Figures/high-speed.PNG>
</div>
<div align=center>
<img src=https://github.com/Qin1143/Multi_AGV_Planner/blob/main/Figures/high-speed.gif>
</div>

### Gazebo simulation
<div align=center>
<img src=https://github.com/Qin1143/Multi_AGV_Planner/blob/main/Figures/gazebo_warehouse.png>
</div>
<div style="display: flex; justify-content: space-between;">
    <img src="https://github.com/Qin1143/Multi_AGV_Planner/blob/main/Figures/gazebo_starts_goals.png" alt="Image 1" width="300"/>
    <img src="https://github.com/Qin1143/Multi_AGV_Planner/blob/main/Figures/gazebo_result_3D.png" alt="Image 2" width="400"/>
</div>

### Multi-AGV planner result
<div align=center>
<img src=https://github.com/Qin1143/Multi_AGV_Planner/blob/main/Figures/STAstar_gx_10agents.png>
</div>

<div align="center">
<table>
  <tr>
    <td>
      <figure>
        <img src="https://github.com/Qin1143/Multi_AGV_Planner/blob/main/Figures/10agents_redom.png" alt="Image 1" width="400"/>
        <figcaption>10 Agents</figcaption>
      </figure>
    </td>
    <td>
      <figure>
        <img src="https://github.com/Qin1143/Multi_AGV_Planner/blob/main/Figures/20agents_redom.png" alt="Image 2" width="400"/>
        <figcaption>20 Agents</figcaption>
      </figure>
    </td>
  </tr>
  <tr>
    <td>
      <figure>
        <img src="https://github.com/Qin1143/Multi_AGV_Planner/blob/main/Figures/30agents_redom.png" alt="Image 3" width="400"/>
        <figcaption>30 Agents</figcaption>
      </figure>
    </td>
    <td>
      <figure>
        <img src="https://github.com/Qin1143/Multi_AGV_Planner/blob/main/Figures/40agents_redom.png" alt="Image 4" width="400"/>
        <figcaption>40 Agents</figcaption>
      </figure>
    </td>
  </tr>
</table>
</div>