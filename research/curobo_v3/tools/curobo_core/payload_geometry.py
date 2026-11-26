"""NumPy reference and tensor full-size payload geometry; all 15 SAT axes and thresholds are unchanged."""
import numpy as np


def box_sphere_depth(centers,radii,positions,rotations,half):
    local=np.einsum('bsi,bij->bsj',centers-positions[:,None],rotations)
    delta=np.abs(local)-np.asarray(half)
    distance=np.linalg.norm(np.maximum(delta,0),axis=-1)+np.minimum(delta.max(-1),0)
    return radii-distance


def box_overlap_depth(first_position,first_rotation,first_half,second_position,second_rotation,second_half):
    batch=len(first_position)
    other=np.broadcast_to(second_rotation,(batch,3,3))
    axes=np.concatenate((first_rotation.transpose(0,2,1),other.transpose(0,2,1),
        np.cross(first_rotation.transpose(0,2,1)[:,:,None],other.transpose(0,2,1)[:,None]).reshape(batch,9,3)),axis=1)
    lengths=np.linalg.norm(axes,axis=-1)
    normalized=axes/np.maximum(lengths[:,:,None],1e-15)
    first_extent=np.abs(normalized@first_rotation)@np.asarray(first_half)
    second_extent=np.abs(normalized@other)@np.asarray(second_half)
    separation=np.abs(np.einsum('bi,bai->ba',np.asarray(second_position)-first_position,normalized))
    overlap=first_extent+second_extent-separation
    overlap[lengths<1e-10]=np.inf
    return overlap.min(-1)


def box_overlap_depth_tensor(centers, rotations, halves, other_centers, other_rotations, other_halves):
    """Signed batched SAT, near-parallel flag, and stable face-axis separation.

    Call with float64. Unlike optimizer guidance, preserve negative separation and
    the CPU audit's 1e-10 axis cutoff; do not apply an activation margin here.
    """
    import torch
    axes_a = rotations.transpose(-1, -2).unsqueeze(-3)
    axes_b = other_rotations.transpose(-1, -2).expand(*centers.shape[:-1], -1, -1, -1)
    axes_a = axes_a.expand_as(axes_b)
    cross = torch.linalg.cross(axes_a.unsqueeze(-2), axes_b.unsqueeze(-3), dim=-1)
    axes = torch.cat((axes_a, axes_b, cross.flatten(-3, -2)), dim=-2)
    lengths = torch.linalg.vector_norm(axes, dim=-1)
    unit = axes/lengths.clamp(min=1e-15).unsqueeze(-1)
    separation = ((centers.unsqueeze(-2)-other_centers).unsqueeze(-2)*unit).sum(-1).abs()
    extent_a = (torch.matmul(unit, rotations.unsqueeze(-3)).abs()*halves.unsqueeze(-2).unsqueeze(-2)).sum(-1)
    extent_b = (torch.matmul(unit, other_rotations).abs()*other_halves.unsqueeze(-2)).sum(-1)
    overlap = extent_a+extent_b-separation
    depth = torch.where(lengths >= 1e-10, overlap, float('inf')).amin(-1)
    uncertain = (lengths[..., 6:] < 1e-6).any(-1)
    return depth, uncertain, overlap[..., :6].amin(-1)


def box_overlap_depth_projected_tensor(centers, rotations, halves, other_centers, other_rotations, other_halves):
    """Search-only15-axis SAT for orthonormal OBB frames; retain ambiguity fallback.

    This uses projections in each box frame instead of materializing every
    normalized axis projection. The explicit reference above stays unchanged.
    Near-parallel/threshold decisions still go through the original CPU audit.
    """
    import torch
    relation = rotations.transpose(-1,-2).unsqueeze(-3) @ other_rotations
    absolute = relation.abs()
    delta = other_centers-centers.unsqueeze(-2)
    local_a = (delta.unsqueeze(-2) @ rotations.unsqueeze(-3)).squeeze(-2)
    local_b = (delta.unsqueeze(-2) @ other_rotations).squeeze(-2)
    face_a = halves.unsqueeze(-2) + (absolute @ other_halves.unsqueeze(-1)).squeeze(-1) - local_a.abs()
    face_b = (absolute.transpose(-1,-2) @ halves.unsqueeze(-2).unsqueeze(-1)).squeeze(-1) + other_halves - local_b.abs()
    extent_a = (halves.roll(-1,-1).unsqueeze(-2).unsqueeze(-1)*absolute.roll(1,-2)
                + halves.roll(1,-1).unsqueeze(-2).unsqueeze(-1)*absolute.roll(-1,-2))
    extent_b = (other_halves.roll(-1,-1).unsqueeze(-2)*absolute.roll(1,-1)
                + other_halves.roll(1,-1).unsqueeze(-2)*absolute.roll(-1,-1))
    separation = (local_a.roll(1,-1).unsqueeze(-1)*relation.roll(-1,-2)
                   - local_a.roll(-1,-1).unsqueeze(-1)*relation.roll(1,-2)).abs()
    axes_b = other_rotations.transpose(-1,-2).expand(*centers.shape[:-1],-1,-1,-1)
    axes_a = rotations.transpose(-1,-2).unsqueeze(-3).expand_as(axes_b)
    lengths = torch.linalg.vector_norm(torch.linalg.cross(axes_a.unsqueeze(-2),axes_b.unsqueeze(-3),dim=-1),dim=-1)
    cross_depth = (extent_a+extent_b-separation)/lengths.clamp(min=1e-15)
    cross_depth = torch.where(lengths>=1e-10,cross_depth,float('inf')).flatten(-2)
    face_depth = torch.cat((face_a,face_b),dim=-1).amin(-1)
    depth = torch.minimum(face_depth,cross_depth.amin(-1))
    uncertain = (lengths<1e-6).any(-1).any(-1)
    return depth,uncertain,face_depth
