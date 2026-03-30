#!/usr/bin/env python3

# xmin xmax ymin ymax 
MOCK_REGIONS = [
    ((-1.1, 1.1, -2.4, 0.2), 'red_bathroom'),
    ((-1.1, 1.1, 1.5, 3.45), 'green_cubicle'),
    ((3, 5.7, 1.5, 3.45), 'blue_shelf'),
    ((3, 5.7, -2.4, 0.2), 'teal_meeting_room'),
]

""" Return a label based on robot position in the simulated world."""
def mock_label_image(robot_x: float, robot_y: float) -> str | None:
    for (xmin, xmax, ymin, ymax), label in MOCK_REGIONS:
        if xmin <= robot_x <= xmax and ymin <= robot_y <= ymax:
            return label
    return None

def mock_embedding(text: str) -> list[str]:
    """Return a trivial character-based embedding.

    For the real thing:
        import clip
        model, preprocess = clip.load("ViT-B/32", device=device)
        image_features = model.encode_image(latest_view)
        text_features = model.encode_text(text)
        # use the features to get a similarity score
        logits_per_image, logits_per_text = model(image, text)
        probs = logits_per_image.softmax(dim=-1).cpu().numpy()
    """

    vec = [0.0] * 26
    for ch in text.lower():
        if 'a' <= ch <= 'z':
            vec[ord(ch) - ord('a')] += 1.0
    norm = sum(v * v for v in vec) ** 0.5
    return [v / norm for v in vec] if norm > 0 else vec

def cosine_similarity(a: list[float], b: list[float]) -> float:
    dot = sum(x * y for x, y in zip(a, b))
    na = sum(x * x for x in a) ** 0.5
    nb = sum(x * x for x in b) ** 0.5
    return dot / (na * nb) if na and nb else 0.0