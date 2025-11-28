This project involves designing a robust LQR controller for a hexacopter, designed to work despite loss of thrust in one of the motors.

<table>
  <tr>
    <th align="center">Nominal LQR controller K₀</th>
    <th align="center">Robust LQR controller Kᵣ</th>
  </tr>
  <tr>
    <td align="center">
      <img src="https://github.com/c-hars/Robust-LQR/blob/main/animation/animation_K0.gif"><br>
    </td>
    <td align="center">
      <img src="https://github.com/c-hars/Robust-LQR/blob/main/animation/animation_Kr.gif"><br>
      (See the <a href="https://github.com/c-hars/Robust-LQR/blob/main/animation/animation_Kr.mp4">mp4</a> animation for better quality)
    </td>
  </tr>
</table>

The robust LQR controller is designed using the Linear Matrix Inequality outlined in *Extended H 2 and H*$\infty$ *norm characterizations and controller parametrizations for discrete-time systems* (MC De Oliveira, JC Geromel, J Bernussou, 2002).

The design process can be [downloaded](https://github.com/c-hars/Robust-LQR/blob/main/notebook.mlx) and explored in MATLAB, or [viewed in the browser](https://github.com/c-hars/Robust-LQR/blob/main/notebook.md).