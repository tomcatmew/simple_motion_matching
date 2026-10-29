# Elderly and assisted walking (100STYLE)

These clips come from the **100STYLE dataset** by Ian Mason, Sebastian Starke and Taku Komura:
<https://www.ianxmason.com/100style/>. They were captured with an Xsens suit at 60 fps.

| Style | What it shows |
|---|---|
| `Neutral` | an ordinary walk, as a reference |
| `Old` | elderly gait: a stooped posture, short and slow steps, a careful balance |
| `WalkingStickLeft` / `WalkingStickRight` | walking with a cane in the left / right hand |
| `LimpLeft` / `LimpRight` | limping on the left / right leg |

Each style has three clips:

- `FW`: forward walking with turns (the first 40 s)
- `TR1`: transitions such as starting, stopping, speeding up and changing direction (the first 40 s)
- `ID`: standing idle

Open them with the motion viewer (`Motion_viewer`), or load any BVH tool.

## License and changes

The 100STYLE dataset is licensed under [CC BY 4.0](https://creativecommons.org/licenses/by/4.0/).
Credit: *The 100STYLE Dataset - Ian Mason*.

```
@article{mason2022local,
  author  = {Mason, Ian and Starke, Sebastian and Komura, Taku},
  title   = {Real-Time Style Modelling of Human Locomotion via Feature-Wise Transformations and Local Motion Phases},
  journal = {Proceedings of the ACM on Computer Graphics and Interactive Techniques},
  volume  = {5}, number = {1}, articleno = {6}, year = {2022}, doi = {10.1145/3522618}
}
```

The files here are modified: frames outside the dataset's `Frame_Cuts.csv` are removed, `FW` and `TR1` are cut
to their first 40 s, and the values are rounded to 0.01 (joint positions change by less than 0.4 mm).
The skeleton is unchanged. To download the originals and recreate these files, run `python tools/fetch_100style.py`.
