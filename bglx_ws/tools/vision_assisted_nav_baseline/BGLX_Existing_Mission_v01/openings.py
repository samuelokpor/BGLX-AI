"""2D lidar recess hypotheses, not traversability or exit certification."""
import numpy as np

def find_openings(ranges, angle_min, angle_increment, range_min, range_max,
                  jump_m=1.5, min_width_m=.9, max_width_m=6.):
    r=np.asarray(ranges,dtype=float)
    if angle_increment<=0: raise ValueError('Positive scan increment required')
    valid=np.isfinite(r)&(r>range_min)&(r<range_max)
    # Invalid readings can lie inside a hypothesis but never define its jambs.
    far=np.where(valid,r,np.inf)
    starts=[j for j in range(1,len(r)-3) if valid[j] and valid[j-1]
            and abs(r[j]-r[j-1])<.1 and far[j+3]>r[j]+jump_m]
    ends=[j for j in range(3,len(r)-1) if valid[j] and valid[j+1]
          and abs(r[j]-r[j+1])<.1 and far[j-3]>r[j]+jump_m]
    found=[]
    for left in starts:
        for right in ends:
            if right<=left+2: continue
            span=(right-left)*angle_increment
            if span>1.4: break
            angles=angle_min+np.array([left,right])*angle_increment
            jambs=r[[left,right],None]*np.column_stack((np.cos(angles),np.sin(angles)))
            width=float(np.linalg.norm(jambs[1]-jambs[0]))
            if not min_width_m<=width<=max_width_m: continue
            mid=jambs.mean(axis=0);tangent=jambs[1]-jambs[0]
            normal=np.array([-tangent[1],tangent[0]])/width
            if normal@mid<0: normal=-normal
            indices=np.arange(left+1,right)
            aa=angle_min+indices*angle_increment
            rays=np.column_stack((np.cos(aa),np.sin(aa)))
            denom=rays@normal
            if np.any(denom<=.1): continue
            wall_range=float(mid@normal)/denom
            finite=valid[indices]
            # Require some farther measured returns; all-missing spans stay unknown.
            if finite.sum()<2: continue
            if np.any(r[indices][finite]<wall_range[finite]-.2): continue
            farther=finite & (r[indices]>wall_range+jump_m)
            if farther.sum()<2: continue
            if (farther.sum()+(~finite).sum())/len(indices)<.6: continue
            if any(abs(left-f['indices'][0])<=3 for f in found): break
            found.append(dict(jambs_scan_xy=jambs.tolist(),mouth_scan_xy=mid.tolist(),
                outward_normal_scan_xy=normal.tolist(),width_at_scan_height_m=width,
                angular_span_rad=float(span),far_return_count=int(finite.sum()),
                missing_return_count=int((~finite).sum()),indices=[int(left),int(right)]))
            break
    return found
