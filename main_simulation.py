import matplotlib.pyplot as plt
from stewart import load_and_filter_motion, animate_simulation

if __name__ == "__main__":

    print("  STEWART PLATFORM MOTION SIMULATION")

    
    try:
       # ladda filtered trajectory f r n motion data
       #  # ndra horse_name and segment_idx om det b e h v s :
        # - horse_name : " Baldur " , " Albin " , " Sigge "( mekaniska simulationen av Baldur ) , " Sam "( mekaniska simualtionen av Albin )
        # - segment_idx : 0 -6 ( beroende p vilket t l t segmentman vill kolla p )
        
        trajectory = load_and_filter_motion(horse_name="Sigge", segment_idx=0)
        
        # Animate the platform following the motion
        fig, anim = animate_simulation(trajectory)
        plt.tight_layout()
        plt.show()
        
    except Exception as e:
        import traceback
        traceback.print_exc()
