import pymel.core as pm


# Rename ALL items in scene...
# To match, thing_side_fluff_more_fluff_01(_end)
for item in pm.ls():
    seperator = '_'
    old_name = item.nodeName()
    split_name = old_name.split(seperator)

    good_side_names = {'left': 'L', 'right': 'R', 'center': 'C'}

    # Check if joint and last in hierarchy
    if isinstance(item, pm.datatypes.Joint) and not item.nodeName().endswith('end'):
        if not item.getChildren():
            # Prompt user to change name...
            opt = raw_input("{} does not end with 'end'. Press 'y' to change this.".format(item.nodeName()))
            if opt.upper() = 'Y':
                split_name[-1] = 'end'

    # Check for bad side names and replace with good side names.
    _split_name = [x.lower() for x in split_name]

    bad_side_in_split_name = {'left', 'right', 'center'}
    bad_side_in_split_name.intersection_update(set(_split_name))

    if bad_side_in_split_name:
        side = bad_side_in_split_name.pop()
        i = _split_name.index(side)
        split_name[i] = good_side_names[side]

    # Check that side isn't first name, if so switch places with whatever is next to it.
    if split_name[0] in ['L', 'R', 'C']:
        split_name[0], split_name[1] = split_name[1], split_name[0]

    name = '_'.join(split_name)

    pm.rename(item, name)