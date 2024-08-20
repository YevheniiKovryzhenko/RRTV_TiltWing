function rmpath(fld)
path_list_cell = regexp(path,';','Split');
if any(ismember(fld, path_list_cell))
    rmpath(fld)
end
end

